from flask import Flask, request, jsonify
import requests
import time
import threading
import rospy
import os
from pallet_pick_and_place.srv import *
from std_msgs.msg import Int64, Float32MultiArray

# creating a Flask app 
app = Flask(__name__)

executer_url = 'http://localhost:9091/execution'
threading.Thread(target=lambda: rospy.init_node('flask_server2', disable_signals=True, anonymous=True)).start()

def yk_robots (start_msg):
    if "robotArm1" in start_msg['resources']:
        yk_builder_pallet_pick_and_place = rospy.ServiceProxy('/yk_builder/pallet_pick_and_place', PalletPickAndPlace)
        yk_builder_assembly_task_pub = rospy.Publisher('/yk_builder/assembly_task', Int64, queue_size=1)
        yk_builder_assembly_plate_state_pub = rospy.Publisher('/yk_builder/assembly_plate_state', Float32MultiArray, queue_size=1)
        yk_builder_kit_plate_state_pub = rospy.Publisher('/yk_builder/kit_plate_state', Float32MultiArray, queue_size=1)
        yk_builder_start_task_pub = rospy.Publisher('/yk_builder/start_task', Int64, queue_size=1)
        yk_builder_task_type_pub = rospy.Publisher('/yk_builder/task_type', Int64, queue_size=1)
        rospy.sleep(1)

        if 'amr1' in start_msg['resources']:
            amr_num = 1
        elif 'amr2' in start_msg['resources']:
            amr_num = 2

        if "loadKit" in start_msg['name'] or "loadStructure" in start_msg['name']:
            rospy.wait_for_service('/yk_builder/pallet_pick_and_place')
            if "unloadKit" in start_msg['name']:
                resp = yk_builder_pallet_pick_and_place(2, False, amr_num)
            elif "loadKit" in start_msg['name']:
                resp = yk_builder_pallet_pick_and_place(2, True, amr_num)
            elif "unloadStructure" in start_msg['name']:
                resp = yk_builder_pallet_pick_and_place(1, False, amr_num)
            elif "loadStructure" in start_msg['name']:
                resp = yk_builder_pallet_pick_and_place(1, True, amr_num)

            if resp.success:
                complete_msg = {'msgType': 'EndTask', 'taskId': start_msg['taskId'], 'name': start_msg['name'], 'outcome': 'success'}
            else:
                complete_msg = {'msgType': 'EndTask', 'taskId': start_msg['taskId'], 'name': start_msg['name'], 'outcome': 'failure'}
        elif "assembleStructure" in start_msg['name']:

            if start_msg['structureType'] == 'human':
                yk_builder_assembly_task_pub.publish(0)
            elif start_msg['structureType'] == 'heart':
                yk_builder_assembly_task_pub.publish(1)
            elif start_msg['structureType'] == 'stairs':
                yk_builder_assembly_task_pub.publish(2)
                
            if "disassembleStructure" in start_msg['name']:
                yk_builder_task_type_pub.publish(0)
            elif "assembleStructure" in start_msg['name']:
                yk_builder_task_type_pub.publish(1)

            print('here')

            yk_builder_assembly_plate_state_pub.publish(Float32MultiArray(data=[0.2645, 0.0465, 0.193, 0, 0, -0.0114]))
            yk_builder_kit_plate_state_pub.publish(Float32MultiArray(data=[0.2598, 0.448, 0.192, 0, 0, 0]))
            rospy.sleep(1)
            yk_builder_start_task_pub.publish(1)
            rospy.sleep(1)
            yk_builder_start_task_pub.publish(0)

            task_running = True
            while task_running:
                task_running = rospy.wait_for_message('/yk_builder/execution_status', Int64).data
            
            complete_msg = {'msgType': 'EndTask', 'taskId': start_msg['taskId'], 'name': start_msg['name'], 'outcome': 'success'}
        
    elif "robotArm2" in start_msg['resources']:
        yk_creator_pallet_pick_and_place = rospy.ServiceProxy('/yk_creator/pallet_pick_and_place', PalletPickAndPlace)
        yk_creator_assembly_task_pub = rospy.Publisher('/yk_creator/assembly_task', Int64, queue_size=1)
        yk_creator_assembly_plate_state_pub = rospy.Publisher('/yk_creator/assembly_plate_state', Float32MultiArray, queue_size=1)
        yk_creator_kit_plate_state_pub = rospy.Publisher('/yk_creator/kit_plate_state', Float32MultiArray, queue_size=1)
        yk_creator_start_task_pub = rospy.Publisher('/yk_creator/start_task', Int64, queue_size=1)
        yk_creator_task_type_pub = rospy.Publisher('/yk_creator/task_type', Int64, queue_size=1)
        rospy.sleep(1)

        if 'amr1' in start_msg['resources']:
            amr_num = 1
        elif 'amr2' in start_msg['resources']:
            amr_num = 2

        if "loadKit" in start_msg['name'] or "loadStructure" in start_msg['name']:
            rospy.wait_for_service('/yk_creator/pallet_pick_and_place')
            if "unloadKit" in start_msg['name']:
                resp = yk_creator_pallet_pick_and_place(2, False, amr_num)
            elif "loadKit" in start_msg['name']:
                resp = yk_creator_pallet_pick_and_place(2, True, amr_num)
            elif "unloadStructure" in start_msg['name']:
                resp = yk_creator_pallet_pick_and_place(1, False, amr_num)
            elif "loadStructure" in start_msg['name']:
                resp = yk_creator_pallet_pick_and_place(1, True, amr_num)

            if resp.success:
                complete_msg = {'msgType': 'EndTask', 'taskId': start_msg['taskId'], 'name': start_msg['name'], 'outcome': 'success'}
            else:
                complete_msg = {'msgType': 'EndTask', 'taskId': start_msg['taskId'], 'name': start_msg['name'], 'outcome': 'failure'}
        elif "assembleStructure" in start_msg['name']:

            if start_msg['structureType'] == 'human':
                yk_creator_assembly_task_pub.publish(0)
            elif start_msg['structureType'] == 'heart':
                yk_creator_assembly_task_pub.publish(1)
            elif start_msg['structureType'] == 'stairs':
                yk_creator_assembly_task_pub.publish(2)
                
            if "disassembleStructure" in start_msg['name']:
                yk_creator_task_type_pub.publish(0)
            elif "assembleStructure" in start_msg['name']:
                yk_creator_task_type_pub.publish(1)

            yk_creator_assembly_plate_state_pub.publish(Float32MultiArray(data=[0.27483, 0.037, 0.189, 0, 0, 0.00668]))
            yk_creator_kit_plate_state_pub.publish(Float32MultiArray(data=[0.261, 0.442, 0.189, 0, 0, 0.02116]))
            rospy.sleep(1)
            yk_creator_start_task_pub.publish(1)
            rospy.sleep(1)
            yk_creator_start_task_pub.publish(0)

            task_running = True
            while task_running:
                task_running = rospy.wait_for_message('/yk_creator/execution_status', Int64).data
            
            complete_msg = {'msgType': 'EndTask', 'taskId': start_msg['taskId'], 'name': start_msg['name'], 'outcome': 'success'}
    print(complete_msg)
    requests.post(executer_url, json=complete_msg)

@app.route('/execution', methods = ['POST'])
def start_execution():
    start_msg = request.get_json()
    print(start_msg)
    thread = threading.Thread(target=yk_robots, args=(start_msg,))
    thread.start()
    return '', 204

# driver function 
if __name__ == '__main__': 
    app.run(debug=True, port=9089)
