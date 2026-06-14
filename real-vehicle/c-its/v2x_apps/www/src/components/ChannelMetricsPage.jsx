import { ArrowLeft, BarChart3, Clock3, RadioTower, Wifi } from 'lucide-react'
import { useLDM } from '../context/LDMContext'

function formatNumber(value, fractionDigits = 1) {
  const numeric = Number(value)
  if (!Number.isFinite(numeric)) {
    return 'n/a'
  }
  return numeric.toFixed(fractionDigits)
}

function MetricCard({ title, accent, metric }) {
  return (
    <section className="rounded-2xl border border-slate-700/80 bg-slate-950/70 p-4 shadow-[0_0_0_1px_rgba(15,23,42,0.2)] backdrop-blur-sm">
      <div className="mb-3 flex items-center justify-between gap-3">
        <div>
          <div className="text-xs uppercase tracking-[0.2em] text-slate-500">{title}</div>
          <div className="mt-1 text-lg font-semibold text-slate-100">{metric.packet_count ?? 0} packets</div>
        </div>
        <div className="rounded-full px-3 py-1 text-xs font-semibold" style={{ color: accent, background: `${accent}20` }}>
          {formatNumber(metric.packet_rate_per_sec)} pkt/s
        </div>
      </div>

      <div className="grid grid-cols-2 gap-3 text-sm">
        <div className="rounded-xl bg-slate-900/80 p-3">
          <div className="text-xs text-slate-500">Throughput</div>
          <div className="mt-1 font-semibold text-slate-100">{formatNumber(metric.throughput_bytes_per_sec)} bytes/s</div>
        </div>
        <div className="rounded-xl bg-slate-900/80 p-3">
          <div className="text-xs text-slate-500">Average RSSI</div>
          <div className="mt-1 font-semibold text-slate-100">{formatNumber(metric.avg_power_dbm)} dBm</div>
        </div>
        <div className="rounded-xl bg-slate-900/80 p-3">
          <div className="text-xs text-slate-500">Median RSSI</div>
          <div className="mt-1 font-semibold text-slate-100">{formatNumber(metric.median_power_dbm)} dBm</div>
        </div>
        <div className="rounded-xl bg-slate-900/80 p-3">
          <div className="text-xs text-slate-500">Latency window</div>
          <div className="mt-1 font-semibold text-slate-100">{formatNumber((metric.avg_inter_arrival_seconds ?? 0) * 1000)} ms</div>
        </div>
      </div>

      {metric.avg_datarate_mbps !== undefined && (
        <div className="mt-3 rounded-xl bg-slate-900/80 p-3 text-sm">
          <div className="text-xs text-slate-500">Average data rate</div>
          <div className="mt-1 font-semibold text-slate-100">{formatNumber(metric.avg_datarate_mbps)} Mbps</div>
        </div>
      )}

      <div className="mt-3 text-xs text-slate-400">
        Priority distribution: {Object.keys(metric.priorities ?? {}).length > 0 ? JSON.stringify(metric.priorities) : 'n/a'}
      </div>
    </section>
  )
}

export default function ChannelMetricsPage({ onBack }) {
  const { channelMetrics, status } = useLDM()
  const hasMetrics = Boolean(channelMetrics)

  return (
    <div className="min-h-screen w-full bg-[radial-gradient(circle_at_top_left,_rgba(34,211,238,0.16),_transparent_32%),radial-gradient(circle_at_top_right,_rgba(244,63,94,0.12),_transparent_28%),linear-gradient(180deg,_#020617_0%,_#07111f_52%,_#020617_100%)] text-slate-100">
      <div className="mx-auto flex min-h-screen w-full max-w-7xl flex-col gap-6 px-4 py-4 sm:px-6 lg:px-8">
        <header className="flex flex-col gap-4 rounded-3xl border border-slate-700/80 bg-slate-950/65 p-5 shadow-2xl shadow-cyan-950/20 backdrop-blur-sm lg:flex-row lg:items-center lg:justify-between">
          <div className="flex items-center gap-4">
            <div className="flex h-12 w-12 items-center justify-center rounded-2xl bg-cyan-500/15 text-cyan-300">
              <BarChart3 size={24} />
            </div>
            <div>
              <div className="text-xs uppercase tracking-[0.3em] text-cyan-300/70">Channel dashboard</div>
              <h1 className="mt-1 text-2xl font-semibold text-slate-50">Hybrid ITS-G5 and C-V2X metrics</h1>
              <p className="mt-1 text-sm text-slate-400">Live channel statistics streamed through /ws/ldm.</p>
            </div>
          </div>

          <div className="flex items-center gap-3">
            <span className={`inline-flex items-center gap-2 rounded-full border px-3 py-1 text-sm ${status === 'live' ? 'border-emerald-500/30 bg-emerald-500/10 text-emerald-200' : 'border-amber-500/30 bg-amber-500/10 text-amber-200'}`}>
              {status === 'live' ? <Wifi size={14} /> : <RadioTower size={14} />}
              {status}
            </span>
            <button
              type="button"
              onClick={onBack}
              className="inline-flex items-center gap-2 rounded-xl border border-slate-700 bg-slate-900/80 px-4 py-2 text-sm font-semibold text-slate-100 transition-colors hover:border-cyan-500/40 hover:text-cyan-200"
            >
              <ArrowLeft size={16} />
              Back to map
            </button>
          </div>
        </header>

        {!hasMetrics && (
          <section className="rounded-3xl border border-slate-700/80 bg-slate-950/60 p-6 text-center text-slate-400 shadow-lg backdrop-blur-sm">
            <div className="mx-auto flex max-w-md flex-col items-center gap-3">
              <Clock3 size={28} className="text-cyan-300" />
              <div className="text-lg font-semibold text-slate-100">Waiting for channel metrics</div>
              <div className="text-sm">The page will populate as soon as the collector publishes on /channel_metrics.</div>
            </div>
          </section>
        )}

        {hasMetrics && (
          <>
            <section className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
              <div className="rounded-2xl border border-slate-700/80 bg-slate-950/70 p-4">
                <div className="text-xs uppercase tracking-[0.2em] text-slate-500">CBR</div>
                <div className="mt-2 text-3xl font-semibold text-slate-50">{formatNumber(channelMetrics.channel_busy_ratio_percent)}%</div>
                <div className="mt-1 text-sm text-slate-400">{channelMetrics.busy_samples ?? 0} / {channelMetrics.total_samples ?? 0} busy samples</div>
              </div>
              <div className="rounded-2xl border border-slate-700/80 bg-slate-950/70 p-4">
                <div className="text-xs uppercase tracking-[0.2em] text-slate-500">Last MAC</div>
                <div className="mt-2 break-all text-lg font-semibold text-slate-50">{channelMetrics.last_mac ?? 'n/a'}</div>
              </div>
              <div className="rounded-2xl border border-slate-700/80 bg-slate-950/70 p-4">
                <div className="text-xs uppercase tracking-[0.2em] text-slate-500">Window</div>
                <div className="mt-2 text-3xl font-semibold text-slate-50">{formatNumber(channelMetrics.window_duration_seconds)}s</div>
                <div className="mt-1 text-sm text-slate-400">Measurement interval</div>
              </div>
              <div className="rounded-2xl border border-slate-700/80 bg-slate-950/70 p-4">
                <div className="text-xs uppercase tracking-[0.2em] text-slate-500">Tech mix</div>
                <div className="mt-2 text-lg font-semibold text-slate-50">WLAN {channelMetrics.technology_distribution?.wlan ?? 0}</div>
                <div className="text-sm text-slate-400">C-V2X {channelMetrics.technology_distribution?.cv2x ?? 0} · Unspecified {channelMetrics.technology_distribution?.unspecified ?? 0}</div>
              </div>
            </section>

            <section className="grid gap-4 xl:grid-cols-2">
              <MetricCard title="ITS-G5 / WLAN" accent="#22d3ee" metric={channelMetrics.wlan ?? {}} />
              <MetricCard title="C-V2X / LTE-V2X" accent="#f59e0b" metric={channelMetrics.cv2x ?? {}} />
            </section>
          </>
        )}
      </div>
    </div>
  )
}
