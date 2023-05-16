<template>
  <div>
    <!--    <h5>{{ message }}</h5>-->
    <!--    <el-button type="primary" @click="toggleConnection">{{ buttonText }}</el-button>-->
    <r-o-s-topic />
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import ROSTopic from '@/components/ROSComponents/ROStopic'

export default {
  components: { ROSTopic
  },
  data() {
    return {
      message: 'Waiting for ROS data...',
      buttonText: 'Connect',
      ros: null
    }
  },
  methods: {
    toggleConnection() {
      if (this.ros && this.ros.isConnected) {
        this.ros.close()
      } else {
        this.connectToRos()
      }
    },
    connectToRos() {
      this.ros = new ROSLIB.Ros({
        url: process.env.VUE_APP_ROS_URL
      })
      // alert("connecting")
      this.ros.on('connection', () => {
        this.message = 'Connected.'
        this.buttonText = 'Disconnect'
      })

      this.ros.on('error', error => {
        this.message = 'Error connecting to ROS websocket server: ' + error
        this.buttonText = 'Connect'
      })

      this.ros.on('close', () => {
        this.message = 'Connection to ROS websocket server closed.'
        this.buttonText = 'Connect'
      })

      const topic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/chatter',
        messageType: 'std_msgs/String'
      })

      topic.subscribe(message => {
        this.message = message.data
      })
      // Subscribe to /HMI/plan_skeleton
      const planSkeletonTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/HMI/plan_skeleton',
        messageType: 'std_msgs/String'
      })

      planSkeletonTopic.subscribe(message => {
        this.planSkeletonMessage = message.data
      })

      // Subscribe to /HMI/exe_info
      const exeInfoTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/HMI/exe_info',
        messageType: 'std_msgs/String'
      })

      exeInfoTopic.subscribe(message => {
        this.exeInfoMessage = message.data
      })
    }
  }
}
</script>

<style scoped>
</style>
