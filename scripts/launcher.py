#! /usr/bin/env python3
import subprocess
import socket
import yaml
import threading
import shlex
import os
import rospkg
import time
from netifaces import interfaces, ifaddresses, AF_INET

class LauncherClass(object):
    def __init__(self) -> None:
        self.cnf_path = os.path.join(rospkg.RosPack().get_path("visca_ros"), 'config/config.yaml')
        cnf_data = self.load_from_yaml(self.cnf_path)
        print(cnf_data)
    
    def load_from_yaml(self, path):
        data = None
        with open(path, 'r') as stream:
            data = yaml.safe_load(stream)
            self.host = data['host_ip']
            self.port = data['host_port']
            self.remote_run = data['remote_run']
            self.timeout = data['timeout']
            self.configure_eeprom = data['config_eeprom']
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
        subprocess.run('python3 eeprom_config.py', shell=True, capture_output=False)
        print('Done Configuring EEPROM')
        with open(self.cnf_path, 'w') as stream:
            data = yaml.safe_load(stream)
            data['config_eeprom'] = False
        print('EEPROM flag unset success!')
        subprocess.run('sudo reboot', shell=True, capture_output=False)
    
    def bring_up(self):
        if(self.config_eeprom):
            self.config_eeprom()

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