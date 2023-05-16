<template>
  <div>
    <!--    <div ref="paperContainer" class="jointjs-container" />-->
    <div
      id="myChart"
      style="width: 85vw; height: 60vh;"
    />
    <el-dialog
      :visible.sync="dialogVisible"
      :title="currentModuleType ? currentModuleType.toUpperCase() + ' Selection' : ''"
      width="60%"
      @close="dialogVisible = false"
    >
      <el-table
        border
        :data="$store.state.options.options[currentModuleType]"
        :style="{ height: 'auto' }"
        :row-class-name="rowClassName(currentModuleType, ...arguments)"
        @row-click="selectModule(currentModuleType, ...arguments)"
      >
        <el-table-column
          prop="name"
          label="模块"
          width="180"
        />
        <el-table-column
          prop="description"
          label="描述"
        >
          <template slot-scope="scope">
            {{ scope.row.description ? scope.row.description : '-' }}
          </template>
        </el-table-column>
      </el-table>

      <div
        slot="footer"
        class="dialog-footer"
      >
        <el-button @click="dialogVisible = false">
          Cancel
        </el-button>
        <el-button
          type="primary"
          @click="selectModuleAndClose(currentModuleType, selectedRow)"
        >
          Confirm
        </el-button>
      </div>
    </el-dialog>
    <div
      class="actions"
      style="margin-top: 10px;"
    >
      <el-button @click="setDefault">
        默认
      </el-button>
      <el-button
        type="primary"
        @click="submitAll"
      >
        配置
      </el-button>
      <el-button @click="resetAll">
        重选
      </el-button>
    </div>
  </div>
</template>

<script>
import * as joint from 'jointjs'
import dialog from 'element-ui/packages/dialog'
import echarts from 'echarts/lib/echarts'
import 'echarts/theme/macarons'
import { mapActions, mapState } from 'vuex'
export default {
  name: 'ModuleSelectionDiagram',
  data() {
    return {
      // ...
      selectedRow: null,
      dialogVisible: false,
      dialogTitle: null,
      currentModuleType: null,
      options: {
        uav: [
          { name: 'Caged UAV', description: 'Cage-enclosed UAV for safe indoor operation and close proximity to humans and obstacles' },
          { name: 'AMU UAV', description: 'Autonomous Multirotor UAV designed for various applications and payload capabilities' }
        ],
        sensor: [
          { name: 'IMU', description: 'Inertial Measurement Unit for tracking aircraft motion and orientation', selected: false },
          { name: 'GPS', description: 'Global Positioning System for accurate positioning and navigation', selected: false },
          { name: 'Lidar', description: 'Light Detection and Ranging for precise distance measurement and 3D mapping', selected: false },
          { name: 'Camera', description: 'High-resolution visual imaging for object detection, tracking, and recognition', selected: false }
        ],
        venue: [
          { name: 'Indoor', description: 'Indoor motion capture studio with precise tracking for UAV testing and development' },
          { name: 'Outdoor', description: 'Indoor office environment with complex obstacles and varying lighting conditions' },
          { name: 'Unknown', description: 'Outdoor garden area with trees and vegetation for UAV navigation and obstacle avoidance' }
        ],
        motionPlanner: [
          { name: 'BSCP (NUS)', description: 'Bezier Curve based Smooth Control Planner for optimal and smooth UAV trajectories' },
          { name: 'D* Lite (Carnegie Mellon)', description: 'Incremental heuristic search algorithm for dynamic path planning', selected: false },
          { name: 'A* (NUS)', description: 'Incremental heuristic search algorithm' }

        ],
        task: [
          { name: 'RT-LTL (CUHK)', description: '实时线性时态逻辑，适合用于在具有连续动态的室内巡检场景中进行任务规划，如连续运动路径控制和避障' },
          { name: 'Hybrid LTL (CUHK)', description: '混合线性时态逻辑，用于在包含离散和连续动态的室内巡检场景中进行任务规划，如在不同楼层间的巡检任务和多种传感器数据处理' },
          { name: 'Reactive LTL (Duke)', description: '响应式线性时态逻辑，用于在具有不确定条件下的室内巡检场景中进行鲁棒任务规划，如在环境变化或人员活动频繁的区域巡检', selected: false },
          { name: 'Product LTL (KTH)', description: '乘积线性时态逻辑，用于在具有随机动态的室内巡检场景中进行任务规划，如在具有不确定性环境因素（如通风、光照变化等）的巡检任务中进行规划', selected: false }
        ],
        perception: [
          { name: 'YOLO', description: 'Real-time object detection and localization using the You Only Look Once algorithm' },
          { name: 'Faster R-CNN', description: 'Region-based Convolutional Neural Networks for object detection and segmentation', selected: false },
          { name: 'SSD', description: 'Single Shot MultiBox Detector for efficient object detection and localization', selected: false },
          { name: 'RetinaNet', description: 'Focal Loss based object detector for dense and imbalanced object detection', selected: false },
          { name: 'EfficientDet', description: 'Scalable and efficient object detector with a compound scaling method', selected: false },
          { name: 'Deep SORT', description: 'Deep learning-based Simple Online and Realtime Tracking for object tracking', selected: false }
        ]
      },
      selectedModules: {
        uav: null,
        sensor: null,
        venue: null,
        motionPlanner: null,
        task: null,
        perception: null
      },
      form: {
        uavType: null,
        sensorType: null,
        venue: null,
        motionPlanner: null,
        task: null,
        perception: null
      },
      uavModule: null,
      sensorModule: null,
      venueModule: null,
      motionPlannerModule: null,
      taskModule: null,
      perceptionModule: null,
      myChart: null,
      moduleIndexes: {}
    }
  },
  created() {
  },
  computed: {
    ...mapState('options', ['options', 'selectedModules'])

  },
  mounted() {
    // this.initJointJS()
    this.initmyChartJS()
    this.setDefault()
  },
  methods: {
    initmyChartJS() {
      const UAVModule = {
        name: 'UAVModule',
        type: 'group',
        position: [50, 150],
        children: [
          {
            type: 'rect',
            shape: { width: 200, height: 100, r: 45 },
            style: { fill: '#7D2882', stroke: '#ffffff', lineWidth: 4 }
          },
          {
            type: 'text',
            style: { text: 'UAV', fontSize: 16, fontWeight: 'bold', fill: '#ffffff' },
            position: [30, 20]
          },
          {
            type: 'text',
            style: { text: 'UAV Description', fontSize: 4, fill: '#ffffff' },
            position: [10, 50]
          },
          {
            type: 'text',
            style: { text: '无人机模块', fontSize: 17, fill: '#000000' },
            position: [60, 110]
          }
        ]
      }
      this.uavModule = UAVModule
      const SensorModule = {
        name: 'SensorModule',
        type: 'group',
        position: [300, 150],
        children: [
          {
            type: 'rect',
            shape: { width: 200, height: 100, r: 25 },
            style: { fill: '#7D2882', stroke: '#ffffff', lineWidth: 4 }
          },
          {
            type: 'text',
            style: { text: 'Sensor', fontSize: 16, fontWeight: 'bold', fill: '#fff' },
            position: [10, 20]
          },
          {
            type: 'text',
            style: { text: 'Sensor Description', fontSize: 4, fill: '#fff' },
            position: [10, 50]
          },
          {
            type: 'text',
            style: { text: '传感器模块', fontSize: 17, fill: '#333' },
            position: [60, 110]
          }
        ]
      }
      this.sensorModule = SensorModule
      const TaskModule = {
        name: 'TaskModule',
        type: 'group',
        position: [550, 50],
        children: [
          {
            type: 'rect',
            shape: { width: 200, height: 100, r: 25 },
            style: { fill: '#7D2882', stroke: '#ffffff', lineWidth: 4 }
          },
          {
            type: 'text',
            style: { text: 'Task', fontSize: 16, fontWeight: 'bold', fill: '#fff' },
            position: [10, 20]
          },
          {
            type: 'text',
            style: { text: 'Task Description', fontSize: 4, fill: '#fff' },
            position: [10, 50]
          },
          {
            type: 'text',
            style: { text: '任务规划模块', fontSize: 17, fill: '#333' },
            position: [60, 110]
          }
        ]
      }
      this.taskModule = TaskModule
      const motionPlannerModule = {
        name: 'motionPlannerModule',
        type: 'group',
        position: [800, 50],
        children: [
          {
            type: 'rect',
            shape: { width: 200, height: 100, r: 25 },
            style: { fill: '#7D2882', stroke: '#ffffff', lineWidth: 4 }
          },
          {
            type: 'text',
            style: { text: 'Motion', fontSize: 16, fontWeight: 'bold', fill: '#fff' },
            position: [10, 20]
          },
          {
            type: 'text',
            style: { text: 'Motion Planner Description', fontSize: 4, fill: '#fff' },
            position: [10, 50]
          },
          {
            type: 'text',
            style: { text: '运动规划模块', fontSize: 17, fill: '#333' },
            position: [60, 110]
          }
        ] }
      this.motionPlannerModule = motionPlannerModule
      const venueModule = {
        name: 'venueModule',
        type: 'group',
        position: [550, 250],
        children: [
          {
            type: 'rect',
            shape: { width: 200, height: 100, r: 25 },
            style: { fill: '#7D2882', stroke: '#ffffff', lineWidth: 4 }
          },
          {
            type: 'text',
            style: { text: 'Workspace', fontSize: 16, fontWeight: 'bold', fill: '#fff' },
            position: [10, 20]
          },
          {
            type: 'text',
            style: { text: 'Workspace Description', fontSize: 4, fill: '#fff' },
            position: [10, 50]
          },
          {
            type: 'text',
            style: { text: '工作场景', fontSize: 17, fill: '#333' },
            position: [60, 110]
          }
        ] }
      this.venueModule = venueModule
      const perceptionModule = {
        name: 'perceptionModule',
        type: 'group',
        position: [800, 250],
        children: [
          {
            type: 'rect',
            shape: { width: 200, height: 100, r: 25 },
            style: { fill: '#7D2882', stroke: '#ffffff', lineWidth: 4 }
          },
          {
            type: 'text',
            style: { text: 'Perception', fontSize: 16, fontWeight: 'bold', fill: '#fff' },
            position: [10, 20]
          },
          {
            type: 'text',
            style: { text: 'Perception Description', fontSize: 4, fill: '#fff' },
            position: [10, 50]
          },
          {
            type: 'text',
            style: { text: '感知模块', fontSize: 17, fill: '#333' },
            position: [60, 110]
          }
        ] }
      this.perceptionModule = perceptionModule

      const connectionUavSensor = {
        type: 'polyline',
        shape: {
          points: [
            [250, 200], // Update these points based on the actual positions and sizes of the modules
            [300, 200]
          ]
        },
        style: {
          stroke: '#000',
          lineWidth: 2,
          text: '',
          textPosition: 'middle',
          textDistance: 10,
          textFill: '#333',
          fontSize: 12,
          fontWeight: 'bold'
        },
        z: 10
      }
      const connectionSensorTask = {
        type: 'polyline',
        shape: {
          points: [
            [500, 200],
            [550, 100]
          ]
        },
        style: {
          stroke: '#000',
          lineWidth: 2,
          text: '',
          textPosition: 'middle',
          textDistance: 10,
          textFill: '#333',
          fontSize: 12,
          fontWeight: 'bold'
        },
        z: 10
        // ...
      }
      const connectionSensorVenue = {
        type: 'polyline',
        shape: {
          points: [
            [500, 200],
            [550, 300]
          ]
        },
        style: {
          stroke: '#000',
          lineWidth: 2,
          text: '',
          textPosition: 'middle',
          textDistance: 10,
          textFill: '#333',
          fontSize: 12,
          fontWeight: 'bold'
        },
        z: 10
        // ...
      }
      const connectionTaskMotionPlanner = {
        type: 'polyline',
        shape: {
          points: [
            [750, 100],
            [800, 100]

          ]
        },
        style: {
          stroke: '#000',
          lineWidth: 2,
          text: '',
          textPosition: 'middle',
          textDistance: 10,
          textFill: '#333',
          fontSize: 12,
          fontWeight: 'bold'
        },
        z: 10
        // ...
      }
      const connectionVenuePerception = {
        type: 'polyline',
        shape: {
          points: [
            [750, 300],
            [800, 300]
          ]
        },
        style: {
          stroke: '#000',
          lineWidth: 2,
          text: '',
          textPosition: 'middle',
          textDistance: 10,
          textFill: '#333',
          fontSize: 12,
          fontWeight: 'bold'
        },
        z: 10
        // ...
      }

      const myChart = echarts.init(document.getElementById('myChart'), 'macarons')
      this.myChart = myChart
      myChart.setOption({
        graphic: [
          UAVModule,
          SensorModule,
          TaskModule,
          motionPlannerModule,
          venueModule,
          perceptionModule,
          connectionUavSensor,
          connectionSensorVenue,
          connectionSensorTask,
          connectionTaskMotionPlanner,
          connectionVenuePerception
        ]
      })

      myChart.getZr().on('click', (params) => {
        const element = params.target
        if (element && element.parent && element.parent.type === 'group') {
          const moduleName = element.parent.name
          let moduleType

          // Set moduleType based on the clicked element's parent name
          if (moduleName === 'UAVModule') {
            moduleType = 'uav'
          } else if (moduleName === 'SensorModule') {
            moduleType = 'sensor'
          } else if (moduleName === 'TaskModule') {
            moduleType = 'task'
          } else if (moduleName === 'motionPlannerModule') {
            moduleType = 'motionPlanner'
          } else if (moduleName === 'venueModule') {
            moduleType = 'venue'
          } else if (moduleName === 'perceptionModule') {
            moduleType = 'perception'
          } else {
            // If the clicked element's parent is not a module, ignore the click event
            return
          }

          this.openDialog(moduleType)
        }
      })
    },
    initJointJS() {
      const graph = new joint.dia.Graph()

      const paper = new joint.dia.Paper({
        el: this.$refs.paperContainer,
        width: '100%',
        height: '500px',
        model: graph,
        gridSize: 10,
        drawGrid: false,
        interactive: { zoom: true },
        background: {
          color: 'rgba(255,255,255,0.35)'
        }
      })
      paper.on('cell:pointerup', (cellView) => {
        const moduleType = cellView.model.attr('body/moduleType')

        if (moduleType) {
          this.openDialog(moduleType)
        }
      })
      // Create custom elements for your modules

      const ModuleShape = joint.shapes.standard.Rectangle.extend({
        defaults: joint.util.deepSupplement(
          {
            type: 'custom.module',
            attrs: {
              body: {
                rx: 10,
                ry: 10,
                strokeWidth: 2,
                stroke: 'black',
                fill: 'white'
              },
              label: {
                textVerticalAnchor: 'top', // 设置 title 垂直方向为顶部
                textAnchor: 'middle',
                refX: '50%',
                fontSize: 14,
                fontWeight: 'bold', // 将 title 文本设置为粗体
                fill: 'black',
                title: {
                  text: 'Module Title',
                  position: { x: 0, y: -10 }, // 调整 title 的位置
                  style: { 'font-size': 16 } // 设置 title 字体大小
                },
                description: {
                  text: 'Module Description',
                  textVerticalAnchor: 'middle', // 设置 description 垂直方向为中心
                  position: { x: 0, y: 0 },
                  style: { 'font-size': 12 } // 设置 description 字体大小
                }
              }
            }
          },
          joint.shapes.standard.Rectangle.prototype.defaults
        )
      })
      // Create module elements
      const uavModule = new ModuleShape({
        position: { x: 50, y: 125 },
        size: { width: 100, height: 50 },
        attrs: {
          label: {
            title: {
              text: 'Platform',
              style: { 'font-size': 16, 'font-weight': 'bold' } // 设置 title 的样式
            },
            description: {
              text: 'AMU',
              style: { 'font-size': 12 } // 设置 description 的字体大小
            }
          },
          body: { moduleType: 'uav' }
        }
      })
      this.uavModule = uavModule

      const sensorModule = new ModuleShape({
        position: { x: 250, y: 125 },
        size: { width: 100, height: 50 },
        attrs: {
          label: { text: 'Sensor' },
          body: { moduleType: 'sensor' }
        }
      })
      this.sensorModule = sensorModule

      const taskModule = new ModuleShape({
        position: { x: 400, y: 250 },
        size: { width: 150, height: 50 },
        attrs: {
          label: { text: 'Task Module' },
          body: { moduleType: 'task' }
        }
      })
      this.taskModule = taskModule

      const motionPlannerModule = new ModuleShape({
        position: { x: 600, y: 250 },
        size: { width: 150, height: 50 },
        attrs: {
          label: { text: 'Motion Planner' },
          body: { moduleType: 'motionPlanner' }
        }
      })
      this.motionPlannerModule = motionPlannerModule

      const venueModule = new ModuleShape({
        position: { x: 400, y: 50 },
        size: { width: 100, height: 50 },
        attrs: {
          label: { text: 'Workspace' },
          body: { moduleType: 'venue' }
        }
      })
      this.venueModule = venueModule

      const perceptionModule = new ModuleShape({
        position: { x: 600, y: 50 },
        size: { width: 150, height: 50 },
        attrs: {
          label: { text: 'Perception' },
          body: { moduleType: 'perception' }
        }
      })
      this.perceptionModule = perceptionModule

      function setLinkAttributes(link) {
        link.attr({
          line: {
            strokeDasharray: '5,2',
            strokeWidth: 2,
            targetMarker: {
              type: 'path',
              d: 'M 10 -5 0 0 10 5 z'
            }
          },
          connector: {
            name: link.source().id === sensorModule.id ? 'rounded' : 'smooth', // Use rounded connector for Sensor -> Task/Venue, smooth connector otherwise
            args: {
              radius: 15
            }
          }
        })
      }

      // Add links to the graph
      const linkUavSensor = new joint.shapes.standard.Link({
        source: { id: uavModule.id },
        target: { id: sensorModule.id }
      })
      setLinkAttributes(linkUavSensor)

      const linkSensorVenue = new joint.shapes.standard.Link({
        source: { id: sensorModule.id },
        target: { id: venueModule.id }
      })
      setLinkAttributes(linkSensorVenue)

      const linkSensorTask = new joint.shapes.standard.Link({
        source: { id: sensorModule.id },
        target: { id: taskModule.id }
      })
      setLinkAttributes(linkSensorTask)

      const linkTaskMotionPlanner = new joint.shapes.standard.Link({
        source: { id: taskModule.id },
        target: { id: motionPlannerModule.id }
      })
      setLinkAttributes(linkTaskMotionPlanner)

      const linkVenuePerception = new joint.shapes.standard.Link({
        source: { id: venueModule.id },
        target: { id: perceptionModule.id }
      })
      setLinkAttributes(linkVenuePerception)

      graph.addCells([
        uavModule,
        sensorModule,
        venueModule,
        motionPlannerModule,
        taskModule,
        perceptionModule,
        linkUavSensor,
        linkSensorVenue,
        linkSensorTask,
        linkTaskMotionPlanner,
        linkVenuePerception
      ])

      // Add interactivity, events, and customization as needed
    },
    openDialog(moduleType) {
      this.currentModuleType = moduleType
      this.dialogTitle = moduleType.toUpperCase() + ' Selection'
      this.dialogVisible = true
    },

    selectModuleAndClose(moduleType, row) {
      this.selectModule(moduleType, row)
      this.dialogVisible = false
    },
    selectModule(moduleType, row) {
      // this.highlightRow(moduleType, row)
      this.setModuleAttr(moduleType, row)
      // this.resizeModule(moduleType)
      this.selectedRow = row
      this.selectedModules[moduleType] = row
      console.log(this.selectedModules[moduleType])
    },
    rowClassName(moduleType, { row }) {
      if (row && this[moduleType + 'Module'] && row.name === this[moduleType + 'Module'].name) {
        return 'selected'
      }
      return ''
    },
    setModuleAttr(moduleType, row) {
      const module = this[`${moduleType}Module`]
      const graphicOptions = this.myChart.getOption().graphic[0].elements

      // Find the index of the module in the graphic array only if it doesn't exist in the moduleIndexes object
      if (!this.moduleIndexes.hasOwnProperty(module.name)) {
        this.$set(this.moduleIndexes, module.name, graphicOptions.findIndex((item) => item.name === module.name))
      }

      const moduleIndex = this.moduleIndexes[module.name]

      graphicOptions[moduleIndex + 1].style.fill = '#f0aa23'
      graphicOptions[moduleIndex + 2].style.text = row.name
      graphicOptions[moduleIndex + 3].style.text = row.description

      // Trigger chart update
      this.myChart.setOption({
        graphic: {
          elements: graphicOptions
        }
      })
    },
    resizeModule(moduleType) {
      const module = this[`${moduleType}Module`]
      module.resize(150, 70) // Adjust the width and height as needed
    },
    ...mapActions('options', ['updateOptions', 'updateSelectedModules']),
    submitAll() {
      // Emit the currently selected modules to the parent component
      this.$emit('submit-modules', this.selectedModules)
      console.log('selected', this.selectedModules)
    },
    setDefault() {
      // Loop through all module types and select the first option for each
      Object.keys(this.options).forEach((moduleType) => {
        const firstOption = this.options[moduleType][0]
        this.selectModule(moduleType, firstOption)
      })
    },
    resetAll() {
      // Clear the selected modules
      this.selectedModules = {
        uav: null,
        sensor: null,
        venue: null,
        motionPlanner: null,
        task: null,
        perception: null
      },

      // Reset JointJS paper
      this.initmyChartJS()
    }
  }
}
</script>

<style scoped>
.jointjs-container {
  width: 100%;
  height: 300px;
}
.highlight-row {
  background-color: #f0f9eb !important;
}
</style>
