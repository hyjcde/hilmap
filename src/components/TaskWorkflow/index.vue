<template>
  <div>
    <div>
      <el-steps :active="activeStep" class="step-container" finish-status="success" simple>
        <el-step title="功能模块配置" />
        <el-step title="语音指令下达" />
        <el-step title="巡检任务执行" />
      </el-steps>
    </div>
    <div v-if="activeStep === 1" class="module-selection">
      <!-- 用户选择模块的内容 -->
      <module-selection v-if="activeStep === 1" @submit="goToNextStep" />
    </div>
    <div v-if="activeStep === 2" class="task-import">
      <task-import
        :selected-modules="selectedModules"
        :active-step="activeStep"
        @previous="goToPreviousStep"
        @next="goToNextStep"
      />
    </div>

    <div v-if="activeStep === 3" class="task-execution">
      <!--      <el-button type="primary" @click="goToPreviousStep">上一步</el-button>-->
      <!--      <div class="rvizweb-display" style="width: 50vw; height: 50vh; float: right">-->
      <!--        &lt;!&ndash; RVizWeb 的界面 &ndash;&gt;-->
      <!--        <rviz-web-component />-->
      <!--      </div>-->
      <el-drawer>
        <task-import
          :selected-modules="selectedModules"
          :active-step="activeStep"
          @previous="goToPreviousStep"
          @next="goToNextStep"
        />
      </el-drawer>
      <iframe
        src="http://localhost:8080"
        style="width: 100vw !important; height: 100vh !important; "
      />
    </div>
  </div>
</template>

<script>
import { Steps, Step } from 'element-ui'
import ModuleSelection from '@/components/ModuleSelection/index'
import rvizWebComponent from '@/components/ROSComponents/RvizWebComponent'
import TaskImport from '@/components/TaskImport/index'

export default {
  components: {
    'el-steps': Steps,
    'el-step': Step,
    ModuleSelection,
    TaskImport,
    rvizWebComponent
  },

  data() {
    return {
      activeStep: 1,
      selectedModules: null
    }
  },

  mounted() {
    this.themeChange('#7d2882')
  },
  methods: {
    themeChange(val) {
      this.$store.dispatch('settings/changeSetting', {
        key: 'theme',
        value: val
      })
    },
    goToPreviousStep() {
      if (this.activeStep > 1) {
        this.activeStep--
      }
    },
    goToNextStep(selectedModules) {
      if (this.activeStep < 3) {
        this.activeStep++
      }
      if (selectedModules) {
        this.selectedModules = selectedModules
      }
    }
  }
}
</script>

<style scoped>
.step-container {
  /*position: fixed;*/
  top: 0vh;
  left: 0;
  right: 0;
  z-index: 99; /* set a high z-index to ensure it's above other elements */
}
</style>
