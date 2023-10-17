#! /usr/bin/env python3
import subprocess
import socket
import yaml
import threading
import shlex
import os
import rospkg, rospy
import time
from netifaces import interfaces, ifaddresses, AF_INET
from visca_ros.srv import ViscaService, ViscaServiceRequest, ViscaServiceResponse

class LauncherClass(object):
    def __init__(self) -> None:
        self.cnf_path = os.path.join(rospkg.RosPack().get_path("visca_ros"), 'config/config.yaml')
        self.cnf_data = self.load_from_yaml(self.cnf_path)
        print(self.cnf_data)
    
    def load_from_yaml(self, path):
        data = None
        with open(path, 'r') as stream:
            data = yaml.safe_load(stream)
            self.host = data['host_ip']
            self.port = data['host_port']
            self.remote_run = data['remote_run']
            self.timeout = data['timeout']
            self.can_config_eeprom = data['config_eeprom']
            self.is_first_run = data['first_run']
            self.server_name = data['server_name']
            self.digital_output = data['digital_output']
            self.resolution = data['monitoring_mode']
        return data
        
    def is_ros_running(self):
        #return rosgraph.is_master_online()
        output = subprocess.run('ss -lntu | grep :11311', shell=True, capture_output=True) # ss returns 1 if nothing found
        if output.returncode == 0:
            return True
        else:
            return False
    
    def is_ros_running_socket(self):
        # try to ping the master IP
        try:
            socket.setdefaulttimeout(1)
            # if we do not receive data for 1 second except will kick in
            soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_address = (self.host, self.port)
            soc.connect(server_address)
        except OSError as error:
            return False
        else:
            soc.close()
            # closing the connection after the
            # communication with the master is complete
        return True
    
    def get_ip(self):
        try:
            for ifaceName in interfaces():
                if(ifaceName.startswith('eth') or ifaceName.startswith('eno') or ifaceName.startswith('enp')): #check for wired IP
                    IP = [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr':'No IP'}] )]
                    if(IP[0] == 'No IP'):
                        IP = '127.0.0.1'
                    else:
                        IP = IP[0]
                        break
                if (ifaceName.startswith('wlp') or ifaceName.startswith('wlo')): #check for wireless IP
                    IP = [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr':'No IP'}] )]
                    if(IP[0] == 'No IP'):
                        IP = '127.0.0.1'
                    else:
                        IP = IP[0]
                        break
        except Exception:
            IP = '127.0.0.1'
        return IP
    
    def config_eeprom(self):
        res = subprocess.run('python3 eeprom_config.py', shell=True, capture_output=True)
        print('\nDone Configuring EEPROM: \n', res)
        self.cnf_data['config_eeprom'] = False
        with open(self.cnf_path, 'w') as file:
            yaml.dump(self.cnf_data, file)
        print('EEPROM flag unset success!')
        subprocess.run('sudo reboot', shell=True, capture_output=False)
    
    def config_first_run(self):
        try:
            rospy.wait_for_service(self.server_name, timeout=rospy.Duration(30.0))
            viscacam_srvr = rospy.ServiceProxy(self.server_name, ViscaService)
            try:
                srvobj = ViscaServiceRequest()
                srvobj.type = ViscaServiceRequest.SET_DIGITAL_OUTPUT
                srvobj.command = self.digital_output
                res:ViscaServiceResponse = viscacam_srvr(srvobj)
                if(res.ack.data):
                    srvobj.type = ViscaServiceRequest.SET_RESOLUTION
                    srvobj.command = self.resolution
                    res:ViscaServiceResponse = viscacam_srvr(srvobj)
                    if(res.ack.data): #toggle flag and reboot
                        print('\nDone Setting up for First Run: \n', res)
                        self.cnf_data['first_run'] = False
                        with open(self.cnf_path, 'w') as file:
                            yaml.dump(self.cnf_data, file)
                        print('FIRSTRUN flag unset success!')
                        subprocess.run('sudo reboot', shell=True, capture_output=False)
            except rospy.ServiceException as e:
                print('Exception occurred!:, ', e)
        except rospy.ROSException as e:
            print('Visca Server not found!')
    
    def bring_up(self):
        if(self.can_config_eeprom):
            self.config_eeprom()
        if(self.is_first_run):
            self.config_first_run()

        if(not self.remote_run):
            self.th = threading.Thread(target=lambda: subprocess.run('roslaunch visca_ros visca_camera.launch', shell=True, capture_output=False))
            self.th.start()
            self.th.join()
        else:
            success = False
            start_t = time.time()
            while((time.time() - start_t) <= self.timeout):
                print('Checking for ROS Master on %s' % self.host)
                time.sleep(1.0)
                if(self.is_ros_running_socket()):
                    self.alive = True
                    ip = self.get_ip()
                    print('Ip is: ', ip)
                    #exit(1)
                    command = 'export ROS_MASTER_URI=http://' + str(self.host) + ':' + str(self.port) + '/;  export ROS_IP=' + ip +'; roslaunch visca_ros visca_camera.launch' 
                    self.th = threading.Thread(target=lambda: 
                                    subprocess.run(command, shell=True, capture_output=False))
                    self.th.start()
                    success = True
                    break
            if(success):
                self.th.join()
            else:
                print('Failed to bring up Camera Remotely!')

if __name__ == '__main__':
    launch_obj = LauncherClass()
    launch_obj.bring_up()
    launch_obj.check_and_reconnect()