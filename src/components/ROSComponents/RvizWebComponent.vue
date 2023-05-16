<template>
  <div>
    <!-- RViz container goes here -->
    <div id="rvizweb-container" />
    <el-button @click="addWaypoint">
      Add Waypoint
    </el-button>
    <el-button @click="loadUAVTopics">
      Load UAV Topics
    </el-button>
    <el-button @click="resetView">
      Reset View
    </el-button>
  </div>
</template>

<script>
import * as ROS3D from 'ros3d'
import * as ROSLIB from 'roslib'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
export default {
  data() {
    return {
      ros: null,
      viewer: null,
      uavModel: null
    }
  },
  mounted() {
    // this.initRvizWeb()
    this.initRvizWebGIE()
  },
  methods: {
    initRvizWebGIE() {
      // Connect to ROS
      this.ros = new ROSLIB.Ros({
        // url: 'ws://10.13.5.68:9090'
        url: 'ws://172.245.79.240:9090'
      })
      this.ros.on('connection', () => {
        this.$message({
          message: 'Connected to ROS successfully!',
          type: 'success'
        })
      })

      this.ros.on('error', (error) => {
        this.$message({
          message: 'Error connecting to ROS: ' + error,
          type: 'error'
        })
      })
      // Initialize the viewer
      this.viewer = new ROS3D.Viewer({
        divID: 'rvizweb-container',
        background: '#c4cad7',
        width: document.getElementById('rvizweb-container').clientWidth,
        height: document.getElementById('rvizweb-container').clientHeight,
        antialias: false
      })
      const grid = new ROS3D.Grid({
        cellSize: 1, // Change this value according to your grid cell size preference
        num_cells: 150, // Change this value according to your grid size preference
        color: '#231e1e' // Change this value to set the grid color
      })
      this.viewer.addObject(grid)

      this.uavModel = new THREE.Object3D()
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(1, 1, 1),
        new THREE.MeshBasicMaterial({ color: 0x00ff12 })
      )
      this.uavModel.add(mesh)
      // this.viewer.addObject(this.uavModel)

      const controls = new OrbitControls(this.viewer.camera, this.viewer.renderer.domElement)
      controls.enableDamping = true // Add inertia to the controls

      // const odomTopic = new ROSLIB.Topic({
      //   ros: this.ros,
      //   name: '/LaserOdomTopic',
      //   messageType: 'nav_msgs/Odometry'
      // })
      // odomTopic.subscribe((message) => {
      //   // Update the viewer with the new data
      //   console.log(message)
      //   this.uavModel.position.set(
      //     message.pose.pose.position.x,
      //     message.pose.pose.position.y,
      //     message.pose.pose.position.z
      //   )
      //   this.uavModel.quaternion.set(
      //     message.pose.pose.orientation.x,
      //     message.pose.pose.orientation.y,
      //     message.pose.pose.orientation.z,
      //     message.pose.pose.orientation.w
      //   )
      // })
      this.tfClient = new ROSLIB.TFClient({
        ros: this.ros,
        angularThres: 0.01,
        transThres: 0.01,
        rate: 10.0,
        fixedFrame: '/camera_link' // Use the 'world' frame as the fixed frame
      })
      console.log('using the cam frame')

      // Set up the visualization for the rslidar_points topic
      const pointCloud2 = new ROS3D.PointCloud2({
        ros: this.ros,
        tfClient: this.tfClient,
        rootObject: this.viewer.scene,
        topic: '/camera/depth_registered/points',
        material: { size: 0.05, color: 0xff00ff }
      })
      this.viewer.scene.add(pointCloud2)
      const PointsTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/camera/depth_registered/points',
        messageType: 'sensor_msgs/PointCloud2'
      })

      PointsTopic.subscribe((message) => {
        console.log('points message:', message)
      })
      // const rslidarPointsTopic = new ROSLIB.Topic({
      //   ros: this.ros,
      //   name: '/rslidar_points',
      //   messageType: 'sensor_msgs/PointCloud2'
      // })
      //
      // rslidarPointsTopic.subscribe((message) => {
      //   console.log('rslidar_points message:', message)
      // })
      // const arrowMarker = new ROS3D.MarkerClient({
      //   ros: this.ros,
      //   tfClient: this.tfClient,
      //   topic: '/visualization_marker',
      //   rootObject: this.viewer.scene
      // })
      // const markerTopic = new ROSLIB.Topic({
      //   ros: this.ros,
      //   name: '/visualization_marker',
      //   messageType: 'visualization_msgs/Marker'
      // })
      // markerTopic.subscribe((message) => {
      //   // alert('heartbeat message')
      //   console.log('Received marker message:', message)
      // })

      // this.viewer.scene.addObject(arrowMarker)
    },
    addWaypoint() {
      // Code to add a waypoint to the RViz visualization
    },
    loadUAVTopics() {
      // Code to load pre-recorded ROS topics for a UAV
    },
    resetView() {
      // Code to reset the RViz view
    }
  }
}
</script>

<style scoped>
#rvizweb-container {
  position: absolute;
  top: 10vh;
  left: 35vw;
  width: 60vw;
  height: 75vh;
}
</style>
