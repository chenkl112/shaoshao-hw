from pathlib import Path
import json
import sys
import yaml

def main():
    source,runtime=map(Path,sys.argv[1:3])
    robot=yaml.safe_load((source/'web_panel/robot.yaml').read_text(encoding='utf-8'))
    serial=yaml.safe_load((source/'web_panel/serial.navcar.example.yaml').read_text(encoding='utf-8'))
    serial['serial_port']=robot['hardware']['serial_port'];serial['baud']=robot['hardware']['baud']
    target=runtime/'config/serial.navcar.yaml';target.parent.mkdir(exist_ok=True,parents=True)
    # Preserve commissioned runtime calibration on repeated builds.
    if not target.exists():target.write_text(yaml.safe_dump(serial,allow_unicode=True,sort_keys=False),encoding='utf-8')
    lidar_path=runtime/'config/MID360_config.json'
    lidar=json.loads((source/'deployment/config/MID360_config.json').read_text(encoding='utf-8'))
    host=lidar['MID360']['host_net_info']
    for key in ('cmd_data_ip','push_msg_ip','point_data_ip','imu_data_ip'):host[key]=robot['network']['host_ip']
    lidar['lidar_configs'][0]['ip']=robot['network']['lidar_ip']
    if not lidar_path.exists():lidar_path.write_text(json.dumps(lidar,indent=2)+'\n',encoding='utf-8')
    print('Runtime serial config:',target)

if __name__=='__main__':main()
