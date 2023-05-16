<template>
  <div class="drawer-container">
    <div>
      <h2 class="drawer-title">
        任务配置
      </h2>

      <div style="margin: 2vh auto;">
        <!--        <el-button type="primary" style="margin-bottom: 1vh; float: left;" @click="$emit('previous')">Back</el-button>-->
        <el-tooltip
          content="Take-off"
          placement="top"
        >
          <el-button
            type="primary"
            size="small"
            icon="el-icon-caret-top"
            @click="takeoff"
          >
            起飞
          </el-button>
          <!--        <el-button type="primary" icon="el-icon-caret-top" :disabled="activeStep===2" @click="takeoff">Take-off</el-button>-->
        </el-tooltip>

        <el-tooltip
          content="Land"
          placement="top"
        >
          <el-button
            type="primary"
            size="small"
            icon="el-icon-caret-bottom"
            @click="land"
          >
            降落
          </el-button>
          <!--        <el-button type="primary" icon="el-icon-caret-bottom" :disabled="activeStep===2" @click="land">Land</el-button>-->
        </el-tooltip>
        <el-tooltip
          content="kill the sys"
          placement="top"
        >
          <el-button
            type="primary"
            size="small"
            @click="land"
          >
            紧急停车
          </el-button>
          <!--        <el-button type="primary" icon="el-icon-caret-bottom" :disabled="activeStep===2" @click="land">Kill</el-button>-->
        </el-tooltip>
      </div>
      <div
        class="task-import"
        style="margin: 5vh auto;"
      >
        <el-card
          shadow="hover"
          style="border-radius: 12px"
        >
          <div slot="header">
            <span>任务控制</span>
            <!--              <span>Task Control</span>-->
          </div>

          <el-row>
            <el-col :span="24">
              <el-form label-position="top">
                <div style="float: right">
                  <el-button
                    v-if="!isRecording"
                    size="small"
                    type="primary"
                    @click="startRecording"
                  >
                    开始下达
                  </el-button>
                  <!--                    <el-button v-if="!isRecording" type="primary" @click="startRecording">Start Recording</el-button>-->
                  <el-button
                    v-if="isRecording"
                    size="small"
                    type="primary"
                    @click="stopRecording"
                  >
                    停止下达
                  </el-button>
                  <!--                    <el-button v-if="isRecording" type="primary" @click="stopRecording">Stop Recording</el-button>-->
                </div>
                <!--                  <div>
                                        <el-button type="info" @click="playRecord('https://files.catbox.moe/fuexbp.wav')">DT</el-button>
                                        <el-button type="warning" @click="playRecord('https://files.catbox.moe/8l8rqq.wav')">JB</el-button>
                                      </div>-->
                <el-form-item label="任务">
                  <!--                <el-form-item label=" Task">-->
                  <el-input
                    v-model="transcribedText"
                    type="textarea"
                    autosize
                    placeholder="Your task will appear here."
                  />
                </el-form-item>
                <!--                  <el-button type="primary" style="float: right" :disabled="!transcribedText" @click="generatePlan">Generate Plan</el-button>-->

                <!--                  <el-form-item label="Task Plan">-->
                <!--                    <el-input v-model="translatedText" type="textarea" autosize placeholder="Your detailed task here." />-->
                <!--                  </el-form-item>-->
                <el-button
                  type="primary"
                  style="float: right"
                  size="small"
                  :disabled="!transcribedText"
                  @click="translateToLTLonline"
                >
                  转化
                </el-button>
                <!--                <el-button type="primary" style="float: right" :disabled="!transcribedText" @click="convertLTL">Translate</el-button>-->

                <el-form-item label="Parsed LTL Command:">
                  <el-input
                    v-model="ltlCommand"
                    type="textarea"
                    placeholder="转换后的LTL公式，如."
                  />
                </el-form-item>
                <el-button
                  type="primary"
                  size="small"
                  @click="reset"
                >
                  重制
                </el-button>
                <!--                <el-button type="primary" @click="reset">Reset</el-button>-->
                <!--                <el-button type="primary" style="float: right" :disabled="!ltlCommand" @click="executeCommand">Execute</el-button>-->
                <el-button
                  type="primary"
                  style="float: right"
                  :disabled="!ltlCommand"
                  size="small"
                  @click="executeCommand"
                >
                  运行
                </el-button>
                <el-form-item label="任务规划视图">
                  <!--                <el-form-item label=" Task">-->
                  <el-input
                    v-model="planViewer"
                    type="textarea"
                    autosize
                    placeholder="任务规划信息"
                  />
                </el-form-item>
              </el-form>
            </el-col>
          </el-row>
        </el-card>
      </div>
      <div
        v-show="false"
        class="drawer-item"
      >
        <span>Theme Color</span>
        <theme-picker
          style="float: right;height: 26px;margin: -3px 8px 0 0;"
          @change="themeChange"
        />
      </div>

      <div
        v-show="false"
        class="drawer-item"
      >
        <span>Open Tags-View</span>
        <el-switch
          v-model="tagsView"
          class="drawer-switch"
        />
      </div>

      <div
        v-show="false"
        class="drawer-item"
      >
        <span>Fixed Header</span>
        <el-switch
          v-model="fixedHeader"
          class="drawer-switch"
        />
      </div>

      <div
        v-show="false"
        class="drawer-item"
      >
        <span>Sidebar Logo</span>
        <el-switch
          v-model="sidebarLogo"
          class="drawer-switch"
        />
      </div>
    </div>
  </div>
</template>

<script>
import ThemePicker from '@/components/ThemePicker'
import ServerConnection from '@/components/ROSComponents/ServerConnection'
import { Message } from 'element-ui'
import ROSLIB from 'roslib'
import axios from 'axios'
import roslib from 'roslib/src/core'

const ros = new ROSLIB.Ros({
  url: 'ws://172.245.79.240:9090'
})
export default {
  components: { ServerConnection, ThemePicker },
  data() {
    return {
      transcribedText: '',
      translatedText: '',
      ltlCommand: '',
      isRecording: false,
      mediaRecorder: null,
      planViewer: '任务等待执行'
    }
  },
  computed: {
    fixedHeader: {
      get() {
        return this.$store.state.settings.fixedHeader
      },
      set(val) {
        this.$store.dispatch('settings/changeSetting', {
          key: 'fixedHeader',
          value: val
        })
      }
    },
    tagsView: {
      get() {
        return this.$store.state.settings.tagsView
      },
      set(val) {
        this.$store.dispatch('settings/changeSetting', {
          key: 'tagsView',
          value: val
        })
      }
    },
    sidebarLogo: {
      get() {
        return this.$store.state.settings.sidebarLogo
      },
      set(val) {
        this.$store.dispatch('settings/changeSetting', {
          key: 'sidebarLogo',
          value: val
        })
      }
    }
  },
  methods: {
    themeChange(val) {
      this.$store.dispatch('settings/changeSetting', {
        key: 'theme',
        value: val
      })
    },
    async startRecording() {
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ audio: true })
        this.mediaRecorder = new MediaRecorder(stream)
        const audioChunks = []

        this.mediaRecorder.addEventListener('dataavailable', (event) => {
          audioChunks.push(event.data)
        })

        this.mediaRecorder.addEventListener('stop', async() => {
          Message({
            message: 'Processing audio...',
            type: 'info'
          })

          const audioBlob = new Blob(audioChunks, { type: 'audio/webm' })
          await this.transcribeAudio(audioBlob)

          Message({
            message: 'Audio transcribed successfully!',
            type: 'success'
          })
        })

        this.mediaRecorder.addEventListener('error', (event) => {
          console.error('Error recording audio:', event.error)
        })

        this.mediaRecorder.start()
        this.isRecording = true
        Message({
          message: 'Recording started...',
          type: 'info'
        })
      } catch (error) {
        console.error('Error accessing microphone:', error)
        Message({
          message: 'Error accessing microphone',
          type: 'error'
        })
      }
    },

    stopRecording() {
      if (this.mediaRecorder && this.isRecording) {
        this.mediaRecorder.stop()
        this.isRecording = false
        Message({
          message: 'Recording stopped...',
          type: 'info'
        })
      }
    },
    /* takeOff() {
      Message({
        message: 'Taking off...',
        type: 'info'
      })
      if (this.selectedModules.uavType === 1) {
        const service = new ROSLIB.Service({
          ros: ros,
          name: '/iris_0/engage',
          serviceType: 'std_srvs/Empty'
        })

        const request = new ROSLIB.ServiceRequest()

        service.callService(request, (result) => {
          console.log('Take-off successful:', result)
        }, (error) => {
          console.error('Error taking off:', error)
        })
      }
    },*/
    // todo: take-off and landing
    async takeoff() {
      try {
        const response = await axios.post('http://localhost:5001/api/takeoff')
        console.log(response.data.message)
      } catch (error) {
        console.error('Error executing takeoff command:', error)
      }
    },
    land() {
      Message({
        message: 'landing...',
        type: 'info'
      })
      if (this.selectedModules.uavType === 1) {
        // TODO: kill motion planner first
        const killMotionPlanner = () => {
          const service = new ROSLIB.Service({
            ros: ros,
            name: '/iris_0/cpc_reference_publisher_node/kill',
            serviceType: 'std_srvs/Empty'
          })
          const requestData = {
            param1: 'status',
            param2: 'value2'
          }
          const request = new ROSLIB.ServiceRequest(requestData)

          service.callService(request, (result) => {
            console.log('Motion planner killed:', result)
          }, (error) => {
            console.error('Error killing motion planner:', error)
          })
        }

        killMotionPlanner()

        const service = new ROSLIB.Service({
          ros: ros,
          name: '/iris_0/land',
          serviceType: 'std_srvs/Empty'
        })

        const request = new ROSLIB.ServiceRequest()

        service.callService(request, (result) => {
          console.log('Land successful:', result)
        }, (error) => {
          console.error('Error landing:', error)
        })
      }
    },
    async generatePlan() {
      Message({
        message: 'Start Generating...',
        type: 'info'
      })
      try {
        const apiKey = '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'
        const url = 'https://api.openai.com/v1/chat/completions'
        const headers = {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${apiKey}`
        }

        const selectedModulesStr = `
UAV Type: ${this.selectedModules.uavType}
Sensor Type: ${this.selectedModules.sensorType}
Venue: ${this.selectedModules.venue}
Motion Planner: ${this.selectedModules.motionPlanner}
Task Module: ${this.selectedModules.taskModule}
Perception Module: ${this.selectedModules.perceptionModule}
`.trim()

        const body = JSON.stringify({
          model: 'gpt-3.5-turbo',
          messages: [
            {
              role: 'user',
              content: `Please generate a step-by-step detailed plan with no more than 10 steps for the UAV for the following task, making it suitable for further translation into LTL (Linear Temporal Logic) commands:

Task: ${this.transcribedText}
Selected Modules:
${selectedModulesStr}

Note: Please provide each step as a separate sentence, focusing on specific actions and conditions to be fulfilled. Note that it should be the plan for the UAV movement for the task planning. And try to be concise and logical to make it best for the further conversion to Linear Temporal Logic`
            }
          ],
          temperature: 0.7,
          max_tokens: 300
        })

        const response = await fetch(url, {
          method: 'POST',
          headers: headers,
          body: body
        })

        if (response.ok) {
          const data = await response.json()
          if (data.choices && data.choices.length > 0) {
            const detailedPlan = data.choices[0].message.content.trim()
            this.translatedText = detailedPlan
          } else {
            throw new Error('No detailed plan found')
          }
        } else {
          const errorData = await response.json()
          console.error('Error details:', errorData)
          throw new Error(`Error generating plan: ${response.statusText}`)
        }
      } catch (error) {
        console.error('Error generating plan:', error)
        throw error
      }
    },
    async playRecord(url) {
      try {
        const response = await fetch(url)
        if (!response.ok) {
          throw new Error(`Failed to fetch audio: ${response.statusText}`)
        }
        const arrayBuffer = await response.arrayBuffer()
        const audioBlob = new Blob([arrayBuffer], { type: 'audio/wav' })

        // Play the audio to the user
        const audio = new Audio(URL.createObjectURL(audioBlob))
        await audio.play()

        await this.transcribeAudio(audioBlob)
      } catch (error) {
        console.error('Error playing record:', error)
        Message({
          message: 'Error playing record',
          type: 'error'
        })
      }
    },
    async transcribeAudio(blob) {
      const apiKey = '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'
      const url = 'https://api.openai.com/v1/audio/transcriptions'
      const formData = new FormData()

      formData.append('model', 'whisper-1')
      formData.append('file', blob, 'openai.webm')

      const response = await fetch(url, {
        method: 'POST',
        headers: {
          Authorization: `Bearer ${apiKey}`
        },
        body: formData
      })

      if (response.ok) {
        const data = await response.json()
        this.transcribedText = data.text
        Message({
          message: this.transcribedText,
          type: 'success'
        })
      } else {
        throw new Error(`Error transcribing audio: ${response.statusText}`)
      }
    },
    async translateToLTLonline(textWithReplacedLabels) {
      try {
        const apiKey = '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'
        const url = 'https://api.openai.com/v1/chat/completions'
        const headers = {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${apiKey}`
        }

        const body = JSON.stringify({
          model: 'gpt-3.5-turbo',
          messages: [
            {
              role: 'user',
              content: `Please translate the following natural language UAV task command into a concise and accurate Linear Temporal Logic (LTL) formula:

Natural Language Command: ${textWithReplacedLabels}

Note: The resulting LTL formula should only contain the logical expressions and operators and the key words in the task level, without any additional information or explanations.`
            }
          ],
          temperature: 0.7,
          max_tokens: 200
        })

        const response = await fetch(url, {
          method: 'POST',
          headers: headers,
          body: body
        })

        if (response.ok) {
          const data = await response.json()
          if (data.choices && data.choices.length > 0) {
            const ltlTranslation = data.choices[0].message.content.trim()
            this.ltlCommand = ltlTranslation
            return ltlTranslation
          } else {
            throw new Error('No LTL translation found')
          }
        } else {
          const errorData = await response.json()
          console.error('Error details:', errorData)
          throw new Error(`Error translating to LTL: ${response.statusText}`)
        }
      } catch (error) {
        console.error('Error translating to LTL:', error)
        throw error
      }
    },
    async translateToLTL(textWithReplacedLabels) {
      try {
        const response = await fetch('/api/translate_to_ltl', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify({ english_text: textWithReplacedLabels })
        })

        if (response.ok) {
          const data = await response.json()
          const ltlf_formula = data.ltlf_formula
          // 在这里处理 LTLf 公式，例如显示在 Vue 组件中
        } else {
          console.error('Failed to translate to LTLf:', response.statusText)
        }
      } catch (error) {
        console.error('Error occurred while translating to LTLf:', error)
      }
    },
    async convertLTL() {
      try {
        // Replace labels in the transcribed text
        const textWithReplacedLabels = this.replaceLabels(this.transcribedText)

        // Call an external translation API or service here using the textWithReplacedLabels as input
        // For example, use the Google Translate API, DeepL API, or any other translation API
        // Store the result in this.translatedText

        // Example: (Replace this with your actual API call)
        this.ltlCommand = await this.translateToLTL(textWithReplacedLabels)
      } catch (error) {
        console.error('Error translating text:', error)
        Message({
          message: 'Error translating text' + error,
          type: 'error'
        })
      }
    },
    replaceLabels(text) {
      text = text.replace('pond', 'green room')
      text = text.replace('grassland', 'first floor')
      return text
    },
    executeCommand() {
      this.$emit('next')
      // Implement the logic for executing the LTL command
      // Use the ltlCommand data property as input
      // post this.ltlCommand
      axios.post('http://localhost:5001/api/execute_task', { ltl_command: this.ltlCommand })
        .then((response) => {
          console.log('Execute task response:', response.data)
        })
        .catch((error) => {
          console.error('Error executing task:', error)
        })
    },

    reset() {
      this.transcribedText = ''
      this.translatedText = ''
      this.ltlCommand = ''
    }
  }
}
</script>

<style lang="scss" scoped>
.drawer-container {
  padding: 24px;
  font-size: 14px;
  line-height: 1.5;
  word-wrap: break-word;

  .drawer-title {
    margin-bottom: 12px;
    color: rgba(0, 0, 0, .85);
    font-size: 14px;
    line-height: 22px;
  }

  .drawer-item {
    color: rgba(0, 0, 0, .65);
    font-size: 14px;
    padding: 12px 0;
  }

  .drawer-switch {
    float: right
  }
}
</style>
