const state = {
  options: {
    uav: [
      { name: 'Caged UAV', description: 'Cage-enclosed UAV for safe indoor operation and close proximity to humans and obstacles' },
      { name: 'AMU UAV', description: 'Autonomous Multirotor UAV designed for various applications and payload capabilities' }
    ],
    sensor: [
      { name: 'IMU', description: 'Inertial Measurement Unit for tracking aircraft motion and orientation', selected: false },
      { name: 'GPS', description: 'Global Positioning System for accurate positioning and navigation', selected: false },
      { name: 'Lidar', description: 'Light Detection and Ranging for precise distance measurement and 3D mapping', selected: false },
      { name: 'Intel Realsense Depth Camera', description: 'High-resolution visual imaging for object detection, tracking, and recognition', selected: false }
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
  }
}

const mutations = {
  UPDATE_OPTIONS: (state, payload) => {
    state.options[payload.category] = payload.newOptions
  },
  UPDATE_SELECTED_MODULES: (state, payload) => {
    state.selectedModules[payload.category] = payload.newSelectedModule
  }
}

const actions = {
  updateOptions({ commit }, payload) {
    commit('UPDATE_OPTIONS', payload)
  },
  updateSelectedModules({ commit }, payload) {
    commit('UPDATE_SELECTED_MODULES', payload)
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
