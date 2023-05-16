import Rete from 'rete'
import VueRenderPlugin from 'rete-vue-render-plugin'
import SensorTypeComponentVue from './SensorTypeComponent.vue'

class SensorTypeComponent extends Rete.Component {
  constructor() {
    super('Sensor Type')
    this.data.component = SensorTypeComponentVue
  }

  builder(node) {
    const out = new Rete.Output('sensor', 'Sensor Type', VueRenderPlugin.sockets.any)

    node.addOutput(out)
    node.data.value = ''
  }

  worker(node, inputs, outputs) {
    outputs['sensor'] = node.data.value
  }
}

export default SensorTypeComponent
