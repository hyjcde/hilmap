<template>
  <div>
    <el-select
      v-model="value"
      placeholder="Select UAV Type"
      @change="change"
    >
      <el-option
        label="AMU UAV"
        value="amu"
      />
      <el-option
        label="Cage UAV"
        value="cage"
      />
      <el-option
        label="Quadcopter"
        value="quadcopter"
        :disabled="true"
      />
      <el-option
        label="Hexacopter"
        value="hexacopter"
        :disabled="true"
      />
      <el-option
        label="Fixed-Wing"
        value="fixed_wing"
        :disabled="true"
      />
    </el-select>
  </div>
</template>

<script>
export default {
  props: ['readonly', 'emitter', 'ikey', 'getData', 'putData'],
  data() {
    return {
      value: ''
    }
  },
  mounted() {
    this.value = this.getData(this.ikey)
  },
  methods: {
    change(value) {
      this.update()
    },
    update() {
      if (this.ikey) this.putData(this.ikey, this.value)
      this.emitter.trigger('process')
    }
  }
}
</script>
