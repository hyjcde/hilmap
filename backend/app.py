
# System libraries
import os
import subprocess
import sys
import time

# ROS libraries
import roslib
import roslib.message
import roslib.names
import rospkg
import rospy
import rostopic
import yaml
# flask bindings
from flask import Flask, jsonify, request
from flask_cors import CORS
# rqt bindings
from rqt_ltl.srv import rqtWifi
# ROS bindings
from std_msgs.msg import String
from std_srvs.srv import Empty

# ros path bindings 
# ros_workspace = '/path/hyj/catkin_ws/devel/lib/python3/dist-packages'
# sys.path.append(ros_workspace)

# paths
root_dir = r'/home/joseph/yzchen_ws/task_planning/ltl_translate/ltl-amdp/language/models/torch/rqt_IO'
lang_path = os.path.join(root_dir, 'input_lang.txt')
ltl_path = os.path.join(root_dir, 'output_ltl.txt')
buchi_path = os.path.join(root_dir, 'rqt_dfa.gv')

# NOTE: if run real robot, use line 2!
# robot_name = '/iris_0'
robot_name = ''

app = Flask(__name__)
cors = CORS(app, resources={r"/api/*": {"origins": "http://localhost:9527"}})

@app.route('/api/hello', methods=['GET'])
def hello():
    return jsonify({"message": "Hello from Python backend!"}), 200

@app.route('/api/takeoff', methods=['POST'])
def takeoff():
    engage_cmd = 'rosservice call /engage'
    process = subprocess.Popen(engage_cmd.split(), stdout=subprocess.PIPE)
    return jsonify({"message": "Takeoff command executed"}), 200

@app.route('/api/land', methods=['POST'])
def land():
    rbt_type = rospy.get_param('robot_choice')
    if rbt_type != 'Numerical sim':
        killmotion_cmd = 'rosnode kill  '+ robot_name+ '/cpc_reference_publisher_node'
        process = subprocess.Popen(killmotion_cmd.split(), stdout=subprocess.PIPE)   
        land_cmd = 'rosservice call '+ robot_name+ '/land'
        process = subprocess.Popen(land_cmd.split(), stdout=subprocess.PIPE)   
    return jsonify({"message": "Land command executed"}), 200

@app.route('/api/switch_task', methods=['POST'])
def switch_task():
    task_name = request.json['task_name']
    # Implement the logic to switch to the specified task
    return jsonify({"message": f"Switched to task {task_name}"}), 200


@app.route('/api/execute_task', methods=['POST'])
def execute_task():
    ltl_command = request.json['ltl_command']
    task_formula = ltl_command
    messages = []

    task_formula = replace_labels_ltl4task(task_formula)
    if not task_formula:
        messages.append('No task needs to be planned.')
        rospy.set_param('cur_task', 0)
    else:
        task_type = rospy.get_param('task_planner_choice')
        if task_type == 'RT-LTL (CUHK)':
            task_formula = replace_operators_4ltlf(task_formula)

            with open(ltl_path, 'w') as fp:
                fp.write(task_formula)

            ltl2aut_cmd = '/home/joseph/yzchen_ws/env/py3torch/bin/python3 /home/joseph/yzchen_ws/env/LTLf2DFA/ltlf2dfa.py'
            process = subprocess.Popen(ltl2aut_cmd.split(), stdout=subprocess.PIPE)
            output, error = process.communicate()

            test_gv = output
            srv_rqt_wifi_client('', test_gv)

        else:
            task_formula = replace_operators_4ltl(task_formula)

        rospy.set_param('cur_task', task_formula)

    print('input to task planner is: ' + task_formula)

    # Return a response indicating whether the command was executed successfully
    return jsonify({"message": "Execute task executed", "messages": messages}), 200

def replace_labels_ltl4task(self, ltl):
    ltl = ltl.replace('pool', 'spray')
    ltl = ltl.replace('grassland', 'grass')
    return ltl

def replace_operators_4ltlf( ltl):
    ltl = ltl.replace('<>', 'F')
    ltl = ltl.replace('[]', 'G')
    ltl = ltl.replace('||', '|')
    ltl = ltl.replace('&&', '&')
    return ltl
def replace_operators_4ltl(self, ltl):
    ltl = ltl.replace('F', '<>')
    ltl = ltl.replace('G', '[]')
    ltl = ltl.replace('|', '||')
    ltl = ltl.replace('&', '&&')
    return ltl


@app.route('/api/confirm_system', methods=['POST'])
def confirm_system():
    data = request.json
    robot_choice = data.get('robot_choice')
    task_planner_choice = data.get('task_planner_choice')
    workspace_choice = data.get('workspace_choice')
    print(robot_choice)
    message = confirmSYS_cb(robot_choice, task_planner_choice, workspace_choice)
    return jsonify({"message": message}), 200


def confirmSYS_cb(robot_choice, task_planner_choice, workspace_choice):
    remote_config = robot_choice + ';' + task_planner_choice + ';' + workspace_choice
    ret_msg = launch_remote_planner(remote_config)
    return ret_msg

def launch_remote_planner(mode):
    print('launching remote planner')
    ret_msg = srv_rqt_wifi_client(mode, '')
    return ret_msg

@app.route('/api/get_current_plan', methods=['GET'])
def get_current_plan():
    # Implement the logic to get the current plan from ROS
    return jsonify({"current_plan": "current plan is ..."}), 200

@app.route('/api/get_execution_info', methods=['GET'])
def get_execution_info():
    # Implement the logic to get the execution info from ROS
    return jsonify({"execution_info": "execution info is ..."}), 200

@app.route('/api/kill_system', methods=['POST'])
def kill_system():
    # Implement the logic to kill the ROS system
    kill_cmd = 'rosnode kill -a'
    process = subprocess.Popen(kill_cmd.split(), stdout=subprocess.PIPE)
    return jsonify({"message": "System killed"}), 200


def srv_rqt_wifi_client(mode, gv):
    '''
    The client of next_move_planner service(plan_service), request a planned pose sequence
    :param pose_mode: no special use
    :return: a pose sequence after LTL planning
    '''

    try:
        rospy.wait_for_service('rqt_wifi_service')
        launch_sender = rospy.ServiceProxy('rqt_wifi_service', rqtWifi)
        resp = launch_sender(mode, gv)
        return resp.result

    except rospy.ServiceException:
        return ("Service call failed")
    
@app.route('/api/launch_remote_planner', methods=['POST'])
def launch_remote_planner_api():
    data = request.json
    robot_choice = data.get('robot_choice')
    task_planner_choice = data.get('task_planner_choice')
    workspace_choice = data.get('workspace_choice')
    message = confirmSYS_cb(robot_choice, task_planner_choice, workspace_choice)
    return jsonify({"message": message}), 200


def replace_labels_lang(text):
    # 
    return text

def replace_labels_ltl4display(text):
    # 
    return text
@app.route('/api/translate_to_ltl', methods=['POST'])
def translate_to_ltl():
    english_text = request.json["english_text"]
    english_normed = english_text

    if not english_normed:
        planViewer = "Input language is empty."
    else:
        planViewer = "English translated to LTLf"
        with open(lang_path, "w") as fp:
            fp.write(english_normed)

        py3torch_command = '/path/to/your/py3torch/bin/python3 /path/to/your/task_planning/ltl_translate/ltl-amdp/language/models/torch/rqt_service.py'
        process = subprocess.Popen(py3torch_command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        with open(ltl_path, "r") as fp:
            formula = fp.read()
            formula_normed = replace_labels_ltl4display(formula)

    return jsonify({"ltlf_formula": formula_normed, "planViewer": planViewer}), 200


@app.route('/api/kill_system', methods=['POST'])
def kill_system():
    # Your killSYS_cb function logic here
    # You may need to modify the function to handle the request data properly
    # For example, you can use request.json to access the JSON data sent with the request
    return jsonify({"message": "Kill system executed"}), 200

if __name__ == '__main__':
   app.run(debug=True, port=5001)
