import rospy
import subprocess
from remote_server.srv import rqtWifi, rqtWifiResponse

import os
# root_dir = r'/home/dyl/yzchen_ws/task_ws/hmi_misc/'
root_dir = r'/home/joseph/yzchen_ws/task_planning/ltl_translate/ltl-amdp/language/models/torch/rqt_IO'
buchi_path = os.path.join(root_dir, 'dfa_remote.gv')

def remote_launch(req):

    if req.gv != '':
        with open(buchi_path, 'w') as fp:
            fp.write(req.gv)

        print('got dfa: '+ req.gv)
        return rqtWifiResponse('gv saved')

    config_ls = req.status.split(';')
    assert len(config_ls) ==3, 'service input incorrect'

    robot_choice, task_planner_choice, workspace_choice = config_ls
    rospy.set_param('robot_choice', robot_choice)
    rospy.set_param('task_planner_choice', task_planner_choice)
    rospy.set_param('workspace_choice', workspace_choice)

    print('System configuration is: '+ req.status)

    if robot_choice == 'Numerical sim':
        if task_planner_choice == 'RT-LTL (CUHK)':
            planner_script =  'python /home/dyl/yzchen_ws/task_ws/src/ltl_ros/src/RTLTL_py/HMI_RTMQ_plugin.py'
        elif task_planner_choice == 'Reactive  LTL (Duke)':
            planner_script =  'python /home/dyl/yzchen_ws/task_ws/src/ltl_ros/src/RTLTL_py/HMI_RH_plugin.py'
        elif task_planner_choice == 'Product LTL (KTH)': 
            planner_script =  'python /home/dyl/yzchen_ws/task_ws/src/ltl_ros/src/RTLTL_py/HMI_prodAut_plugin.py'
        elif task_planner_choice == 'Hybrid LTL (CUHK)':
            planner_script = 'TBD'
    elif robot_choice == 'AMU UAV':
        if task_planner_choice == 'RT-LTL (CUHK)':
            planner_script = '/home/joseph/yzchen_ws/task_planning/ltl_interface/src/hmi_scripts/motionA2B_RTMQFF.sh'
            # planner_script = 'bash src/rqt_RTLTL_interface/hmi_scripts/motionA2B_RTMQFF.sh'
        elif task_planner_choice == 'Reactive  LTL (Duke)':
            planner_script =  'bash src/rqt_RTLTL_interface/hmi_scripts/motionA2B_react.sh'
        elif task_planner_choice == 'Product LTL (KTH)': 
            planner_script =  'python /home/dyl/yzchen_ws/task_ws/src/ltl_ros/src/RTLTL_py/HMI_prodAut_plugin.py'

    process = subprocess.Popen(planner_script.split(), stdout=subprocess.PIPE)  
    print('launching planner at: '+ planner_script)

    return rqtWifiResponse('Launch success')

def remote_launch_server():
    rospy.init_node('remote_launch_server')
    rospy.Service('rqt_wifi_service', rqtWifi, remote_launch)
    rospy.spin()


if __name__ == '__main__':
    remote_launch_server()