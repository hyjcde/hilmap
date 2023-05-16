
<template>
  <div>
    <el-card style="margin: 5vh; padding: 2vh; border-radius: 25px">
      <h1>巡检系统运行状况</h1>
      <div style="margin-top: 2vh;">
        <el-input
          v-model="url"
          placeholder="输入连接地址，如 ws://172.245.79.240:9090"
          style="width: 20vw;  float: left;"
        />
        <el-button
          type="primary"
          @click="initROSConnection"
        >
          连接
        </el-button>
        <el-table
          :data="topics"
          height="400"
          stripe
          border
          style="width: 100%; margin-top: 20px;"
          :default-sort="{ prop: 'publisherCount', order: 'descending' }"
          @sort-change="onSortChange"
        >
          <el-table-column
            prop="id"
            label="ID"
            width="50"
          />
          <el-table-column
            prop="name"
            label="订阅话题名"
          />
          <el-table-column
            prop="type"
            label="订阅消息类型"
          />
          <el-table-column
            prop="publisherCount"
            label="发布者"
          />
          <el-table-column
            prop="subscriberCount"
            label="订阅者"
          />
        </el-table>
      </div>
    </el-card>
  </div>
</template>

<script>
import { Table, TableColumn } from 'element-ui'
import ROSLIB from 'roslib'
import { watch } from 'vue'
import { logger } from 'runjs/lib/common'

export default {
  name: 'ROSTopic',
  components: {
    [Table.name]: Table,
    [TableColumn.name]: TableColumn
  },

  props: {},
  data() {
    return {
      ros: null,
      url: '',
      viewer: null,
      topics: []
    }
  },
  watch: {
    async topics(newTopics, oldTopics) {
      if (newTopics.length !== oldTopics.length) {
        const topicsInfo = await Promise.all(
          newTopics.map(async(topic, index) => {
            const publishers = await this.getPublishers(topic.name)
            const subscribers = await this.getSubscribers(topic.name)

            return {
              ...topic,
              publisherCount: publishers.length,
              subscriberCount: subscribers.length
            }
          })
        )

        this.topics = topicsInfo
        console.log(topicsInfo)
      }
    }
  },
  watchTopics() {
    watch(
      () => this.topics,
      () => {
        this.$nextTick(() => {
          this.$refs.topicTable.doLayout()
        })
      }
    )
  },
  mounted() {
    // this.initROSConnection()
    this.pollTopics()
  },
  methods: {
    initROSConnection() {
      console.log(process.env.VUE_APP_ROS_URL)
      this.ros = new ROSLIB.Ros({
        url: this.url
      })

      this.ros.on('connection', () => {
        this.$message({
          message: 'Connected to ROS successfully!',
          type: 'success'
        })
        this.fetchTopics()
      })

      this.ros.on('error', (error) => {
        this.$message({
          message: 'Error connecting to ROS: ' + error,
          type: 'error'
        })
      })

      this.ros.on('close', () => {
        this.$message({
          message: 'Connection to ROS closed',
          type: 'info'
        })
      })
    },
    pollTopics() {
      setInterval(async() => {
        await this.fetchTopics()
      }, 5000) // Poll every 5 seconds (5000 milliseconds)
    },
    async fetchTopics() {
      if (!this.ros) return

      this.ros.getTopics(async(result) => {
        const fetchedTopics = await Promise.all(
          result.topics.map(async(topic, index) => {
            const type = await this.getTopicType(topic)
            return {
              id: index + 1,
              name: topic,
              type,
              // Set publisherCount and subscriberCount to zero or null for now
              publisherCount: 0,
              subscriberCount: 0
            }
          })
        )

        // Compare the previous and current list of topics
        const topicsChanged = JSON.stringify(this.topics) !== JSON.stringify(fetchedTopics)

        // Update the topic list only when there are changes
        if (topicsChanged) {
          this.topics = fetchedTopics
          // Here, you can also update the publisher and subscriber counts if you have access to that data
        }
      })
    },
    publisherCellClass({ row }) {
      if (row.publisherCount > 0) {
        return 'greenish'
      }
      return ''
    },
    onSortChange({ prop, order }) {
      if (order === 'ascending') {
        this.topics.sort((a, b) => (a[prop] < b[prop] ? -1 : 1))
      } else if (order === 'descending') {
        this.topics.sort((a, b) => (a[prop] > b[prop] ? -1 : 1))
      } else {
        // Reset the sorting to the original order
        this.topics.sort((a, b) => (a.id < b.id ? -1 : 1))
      }
    },
    getTopicType(topic) {
      return new Promise((resolve) => {
        this.ros.getTopicType(topic, (type) => {
          resolve(type)
        })
      })
    },

    getPublishers(topic) {
      return new Promise((resolve) => {
        this.ros.getPublishers(topic, (publishers) => {
          resolve(publishers)
        })
      })
    },

    getSubscribers(topic) {
      return new Promise((resolve) => {
        this.ros.getSubscribers(topic, (subscribers) => {
          resolve(subscribers)
        })
      })
    }
  }
}
</script>
