<template>
  <div class="dashboard-editor-container">
    <!--    <github-corner class="github-corner" />-->

    <!--    <panel-group @handleSetLineChartData="handleSetLineChartData" />-->

    <el-row style="background:#fff;padding:16px 16px 0;margin-bottom:32px;">
      <el-col :span="8">
        <line-chart :chart-data="lineChartData.efficiency" />
      </el-col>
      <el-col :span="8">
        <line-chart :chart-data="lineChartData.timeTaken" />
      </el-col>
      <el-col :span="8">
        <line-chart :chart-data="lineChartData.pathLength" />
      </el-col>
    </el-row>

    <el-row :gutter="32">
      <el-col :xs="24" :sm="24" :lg="8">
        <div class="chart-wrapper">
          <raddar-chart />
        </div>
      </el-col>
      <el-col :xs="24" :sm="24" :lg="8">
        <div class="chart-wrapper">
          <pie-chart />
        </div>
      </el-col>
      <el-col :xs="24" :sm="24" :lg="8">
        <div class="chart-wrapper">
          <bar-chart />
        </div>
      </el-col>
    </el-row>

    <!--    <el-row :gutter="8">
      <el-col :xs="{span: 24}" :sm="{span: 24}" :md="{span: 24}" :lg="{span: 12}" :xl="{span: 12}" style="padding-right:8px;margin-bottom:30px;">
        <transaction-table />
      </el-col>
      <el-col :xs="{span: 24}" :sm="{span: 12}" :md="{span: 12}" :lg="{span: 6}" :xl="{span: 6}" style="margin-bottom:30px;">
        <todo-list />
      </el-col>
      <el-col :xs="{span: 24}" :sm="{span: 12}" :md="{span: 12}" :lg="{span: 6}" :xl="{span: 6}" style="margin-bottom:30px;">
        <box-card />
      </el-col>
    </el-row>-->
  </div>
</template>

<script>
import PanelGroup from './components/PanelGroup'
import LineChart from './components/LineChart'
import RaddarChart from './components/RaddarChart'
import PieChart from './components/PieChart'
import BarChart from './components/BarChart'
import TransactionTable from './components/TransactionTable'
import TodoList from './components/TodoList'
import BoxCard from './components/BoxCard'

const lineChartData = {
  efficiency: {
    chartTitle: 'Efficiency',
    xAxisLabels: ['Task 1', 'Task 2', 'Task 3', 'Task 4', 'Task 5', 'Task 6', 'Task 7'],
    expectedData: [95, 90, 85, 80, 92, 88, 96],
    actualData: [93, 87, 84, 79, 89, 85, 94]
  },
  timeTaken: {
    chartTitle: 'Time Taken',
    xAxisLabels: ['Task 1', 'Task 2', 'Task 3', 'Task 4', 'Task 5', 'Task 6', 'Task 7'],
    expectedData: [120, 110, 130, 125, 140, 135, 150],
    actualData: [125, 115, 135, 128, 145, 138, 155]
  },
  pathLength: {
    chartTitle: 'Path Length',
    xAxisLabels: ['Task 1', 'Task 2', 'Task 3', 'Task 4', 'Task 5', 'Task 6', 'Task 7'],
    expectedData: [500, 520, 480, 510, 530, 550, 540],
    actualData: [510, 525, 490, 515, 540, 560, 545]
  }
}

export default {
  name: 'DashboardAdmin',
  components: {
    PanelGroup,
    LineChart,
    RaddarChart,
    PieChart,
    BarChart,
    TransactionTable,
    TodoList,
    BoxCard
  },
  data() {
    return {
      lineChartData: lineChartData
    }
  },
  methods: {
    handleSetLineChartData(type) {
      this.lineChartData = lineChartData[type]
    }
  }
}
</script>

<style lang="scss" scoped>
.dashboard-editor-container {
  padding: 32px;
  background-color: rgb(240, 242, 245);
  position: relative;

  .github-corner {
    position: absolute;
    top: 0px;
    border: 0;
    right: 0;
  }

  .chart-wrapper {
    background: #fff;
    padding: 16px 16px 0;
    margin-bottom: 32px;
  }
}

@media (max-width:1024px) {
  .chart-wrapper {
    padding: 8px;
  }
}
</style>
