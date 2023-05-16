<template>
  <div>
    <!--    <button @click="modifyOptions">Modify Options</button>-->
    <el-table :data="options.uav" style="margin-bottom: 20vh;">
      <el-table-column prop="name" label="Name" />
      <el-table-column prop="description" label="Description" />
    </el-table>

    <el-table :data="options.sensor" style="margin-bottom: 20vh;">
      <el-table-column prop="name" label="Name" />
      <el-table-column prop="description" label="Description" />
      <el-table-column label="Actions">
        <template slot-scope="scope">
          <el-button type="text" icon="el-icon-plus" size="small" @click="addOption('sensor')">Add</el-button>
        </template>
      </el-table-column>
    </el-table>
  </div>

</template>

<script>
import { mapState, mapMutations } from 'vuex'

export default {
  name: 'Documentation',
  data() {
    return {
    }
  },
  computed: {
    ...mapState('options', ['options'])
  },
  methods: {
    ...mapMutations('options', ['updateOptions']),
    modifyOptions() {
      const newOptions = { ...this.options, additionalOption: { name: 'New Option', description: 'A new option added for testing' }}
      this.updateOptions(newOptions)
      console.log(this.options)
    },
    addOption(optionType) {
      const newOption = { name: 'New Option', description: 'A new option added for testing' }
      this.options[optionType].push(newOption)
      this.updateOptions(this.options)
    }
  }
}
</script>

<style lang="scss" scoped>
.documentation-container {
  margin: 50px;
  display: flex;
  flex-wrap: wrap;
  justify-content: flex-start;

  .document-btn {
    flex-shrink: 0;
    display: block;
    cursor: pointer;
    background: black;
    color: white;
    height: 60px;
    padding: 0 16px;
    margin: 16px;
    line-height: 60px;
    font-size: 20px;
    text-align: center;
  }
}
</style>
