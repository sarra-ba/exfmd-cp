import { useState } from 'react'
import ChannelMetricsPage from './components/ChannelMetricsPage'
import LDMMap from './components/LDMMap'

function App() {
  const [view, setView] = useState('map')

  if (view === 'metrics') {
    return <ChannelMetricsPage onBack={() => setView('map')} />
  }

  return <LDMMap onOpenMetrics={() => setView('metrics')} />
}

export default App
