import Rete from 'rete'
import VueRenderPlugin from 'rete-vue-render-plugin'
import UAVTypeComponentVue from './UavTypeComponent'

class UAVTypeComponent extends Rete.Component {
  constructor() {
    super('UAV Type')
    this.data.component = UAVTypeComponentVue
  }

  builder(node) {
    const out = new Rete.Output('uav', 'UAV Type', VueRenderPlugin.sockets.any)

    node.addOutput(out)
    node.data.value = ''
  }

  worker(node, inputs, outputs) {
    outputs['uav'] = node.data.value
  }
}

export default UAVTypeComponent
