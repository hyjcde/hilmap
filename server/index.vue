<template>
  <div>
    <el-card class="overview-container">
      <h1>
        Choose Module here
      </h1>
      <ServerConnection />
      <el-row>
        <el-col :span="8">
          <el-form ref="form" :model="form" label-width="120px">
            <el-form-item label="UAV Type">
              <el-select v-model="form.uavType" placeholder="Select UAV Type">
                <el-option label="AMU UAV" value="amu" />
                <el-option label="Cage UAV" value="cage" />
              </el-select>
            </el-form-item>
            <el-form-item label="Sensor Type">
              <el-select v-model="form.sensorType" placeholder="Select Sensor Type">
                <el-option label="IMU" value="imu" />
                <el-option label="GPS" value="gps" />
              </el-select>
            </el-form-item>
            <el-form-item label="Venue">
              <el-select v-model="form.venue" placeholder="Select Venue">
                <el-option label="Vicon Room" value="vicon" />
                <el-option label="Office" value="office" />
                <el-option label="Garden" value="garden" />
                <el-option label="Road" value="road" />
              </el-select>
            </el-form-item>
            <el-form-item label="Motion Planner">
              <el-select v-model="form.motionPlanner" placeholder="Select Motion Planner">
                <el-option label="BSCP (NUS)" value="bscp" />
                <el-option label="Skeloton (ETH)" value="skeloton" />
                <el-option label="Fast (HKUST)" value="fast" />
              </el-select>
            </el-form-item>

            <el-form-item label="Task Module">
              <el-select v-model="form.taskModule" placeholder="Select Task Module">
                <el-option label="Hybrid LTL (CUHK)" value="hybrid" />
                <el-option label="Reactive LTL (Duke)" value="reactive" />
              </el-select>
            </el-form-item>

            <el-form-item label="Perception Module">
              <el-select v-model="form.perceptionModule" placeholder="Select Perception Module">
                <el-option label="YOLO" value="yolo" />
              </el-select>
            </el-form-item>

            <el-button type="primary">Submit</el-button>

          </el-form>
        </el-col>

      </el-row>
    </el-card>
  </div>
</template>
<script>
import ServerConnection from '@/components/ROSComponents/ServerConnection'
export default {
  name: 'ModuleSelection',
  components: {
    ServerConnection
  },
  data() {
    return {
      form: {
        uavType: '',
        sensorType: '',
        venue: '',
        motionPlanner: '',
        taskModule: '',
        perceptionModule: []
      }
    }
  }
}

</script>

<style scoped>

.overview-container {
  margin: 20px;
  /*text-align: center;*/
  padding: 20px auto;
}

.h1 {
  text-align: center;
  font-size: 28px;
  font-weight: bold;
  margin-bottom: 20px;
}

.el-form-item__label {
  font-size: 16px;
  font-weight: bold;
}

.el-input {
  width: 100%;
}

.el-button {
  margin-top: 10px;
}

.el-radio-button {
  padding: 5px 20px;
  border-radius: 20px;
  font-size: 14px;
}

.el-checkbox {
  font-size: 14px;
}

.el-checkbox__label {
  padding-left: 10px;
}

@media only screen and (max-width: 576px) {

  .el-form-item__label {
    font-size: 14px;
  }

  .el-radio-button {
    font-size: 12px;
    padding: 5px 10px;
  }

  .el-checkbox {
    font-size: 12px;
  }
}
</style>

