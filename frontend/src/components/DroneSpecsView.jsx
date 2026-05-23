import { useEffect, useState } from 'react'
import { C } from '../utils/colors.js'

const s = {
  root: {
    width: '100%', height: '100%',
    background: C.bg, overflowY: 'auto',
    fontFamily: "'Inter', 'Segoe UI', system-ui, sans-serif",
    color: C.text,
    boxSizing: 'border-box',
    padding: '28px 32px',
  },
  backBtn: {
    display: 'inline-flex', alignItems: 'center', gap: 6,
    padding: '6px 14px', marginBottom: 24,
    background: C.surface, border: `1px solid ${C.border}`,
    borderRadius: 6, color: C.muted, fontSize: 12,
    cursor: 'pointer', fontFamily: 'inherit',
  },
  title: {
    fontSize: 22, fontWeight: 700, color: C.text,
    marginBottom: 4,
  },
  subtitle: {
    fontSize: 12, color: C.muted, marginBottom: 28,
  },
  grid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fill, minmax(280px, 1fr))',
    gap: 16,
  },
  card: {
    background: C.surface, border: `1px solid ${C.border}`,
    borderRadius: 10, padding: '16px 18px',
  },
  cardTitle: {
    fontSize: 10, fontWeight: 700, color: C.muted,
    letterSpacing: 1.6, textTransform: 'uppercase',
    marginBottom: 14,
    paddingBottom: 8,
    borderBottom: `1px solid ${C.border}`,
  },
  row: {
    display: 'flex', justifyContent: 'space-between',
    alignItems: 'center', fontSize: 12,
    padding: '5px 0',
  },
  rowKey: { color: C.muted },
  rowVal: { color: C.text, fontWeight: 600, fontVariantNumeric: 'tabular-nums' },
  tag: {
    display: 'inline-flex', alignItems: 'center',
    padding: '2px 8px', borderRadius: 12,
    fontSize: 11, fontWeight: 600,
    background: C.accent + '18', color: C.accent,
    border: `1px solid ${C.accent}30`,
  },
  loading: {
    display: 'flex', alignItems: 'center', justifyContent: 'center',
    height: '100%', color: C.muted, fontSize: 14,
  },
  error: {
    display: 'flex', alignItems: 'center', justifyContent: 'center',
    height: '100%', color: '#f85149', fontSize: 14,
  },
}

function SpecRow({ label, value, unit }) {
  const display = unit ? `${value} ${unit}` : String(value)
  return (
    <div style={s.row}>
      <span style={s.rowKey}>{label}</span>
      <span style={s.rowVal}>{display}</span>
    </div>
  )
}

function Card({ title, children }) {
  return (
    <div style={s.card}>
      <div style={s.cardTitle}>{title}</div>
      {children}
    </div>
  )
}

export default function DroneSpecsView({ droneName, onBack }) {
  const [specs, setSpecs] = useState(null)
  const [error, setError] = useState(null)

  useEffect(() => {
    if (!droneName) return

    const controller = new AbortController()
    setSpecs(null)
    setError(null)

    fetch(`/drones/${encodeURIComponent(droneName)}`, { signal: controller.signal })
      .then(response => {
        if (!response.ok) {
          throw new Error('БПЛА не найден')
        }
        return response.json()
      })
      .then(setSpecs)
      .catch(error => {
        if (error.name !== 'AbortError') {
          setError(error.message)
        }
      })

    return () => controller.abort()
  }, [droneName])

  if (error) return <div style={s.error}>{error}</div>
  if (!specs) return <div style={s.loading}>Загрузка характеристик…</div>

  return (
    <div style={s.root}>
      <button style={s.backBtn} onClick={onBack}>
        ‹ Назад к карте
      </button>

      <div style={s.title}>{specs.name}</div>
      <div style={s.subtitle}>
        Сельскохозяйственный БПЛА для опрыскивания &nbsp;·&nbsp;
        <span style={s.tag}>{specs.num_rotors} винтов</span>
      </div>

      <div style={s.grid}>

        <Card title="Масса">
          <SpecRow label="Без нагрузки (конструкция + электроника)" value={specs.mass_empty_kg} unit="kg" />
          <SpecRow label="Аккумуляторный блок" value={specs.mass_battery_kg} unit="kg" />
          <SpecRow label="Полный бак (реагент)" value={specs.mass_tank_full_kg} unit="kg" />
          <div style={{ height: 1, background: C.border, margin: '8px 0' }} />
          <SpecRow label="Макс. взлётная масса" value={specs.mass_takeoff_max_kg.toFixed(1)} unit="kg" />
        </Card>

        <Card title="Аккумулятор и энергия">
          <SpecRow label="Ёмкость" value={specs.battery_capacity_wh} unit="Wh" />
          <SpecRow label="Номинальное напряжение" value={specs.battery_voltage_v} unit="V" />
          <SpecRow label="Резерв" value={specs.battery_reserve_pct} unit="%" />
          <SpecRow label="Полезная энергия" value={(specs.battery_capacity_wh * (1 - specs.battery_reserve_pct / 100)).toFixed(0)} unit="Wh" />
          <SpecRow label="Время зарядки" value={specs.battery_charge_time_min} unit="min" />
        </Card>

        <Card title="Мощность висения (диск актуатора)">
          <SpecRow label="При массе без нагрузки" value={specs.power_hover_empty_w} unit="W" />
          <SpecRow label="При полном баке" value={specs.power_hover_full_w} unit="W" />
          <SpecRow label="Насос опрыскивания" value={specs.spray_pump_power_w} unit="W" />
          <div style={{ height: 1, background: C.border, margin: '8px 0' }} />
          <SpecRow label="Макс. мощность при опрыскивании" value={specs.power_hover_full_w + specs.spray_pump_power_w} unit="W" />
        </Card>

        <Card title="Кинематика">
          <SpecRow label="Крейсерская (опрыскивание)" value={specs.speed_cruise_ms} unit="m/s" />
          <SpecRow label="Макс. (перелёт)" value={specs.speed_max_ms} unit="m/s" />
          <SpecRow label="Вертикальная" value={specs.speed_vertical_ms} unit="m/s" />
        </Card>

        <Card title="Модель разворота">
          <SpecRow label="Длительность разворота на 180°" value={specs.turn_duration_s} unit="s" />
          <SpecRow label="Коэффициент мощности при развороте" value={specs.turn_power_factor} unit="× P_hover" />
        </Card>

        <Card title="Система опрыскивания">
          <SpecRow label="Мин. ширина опрыскивания" value={specs.spray_swath_min_m} unit="m" />
          <SpecRow label="Макс. ширина опрыскивания" value={specs.spray_swath_max_m} unit="m" />
          <SpecRow label="Расход" value={specs.spray_flow_rate_lpm} unit="L/min" />
          <SpecRow label="Доза по умолчанию" value={specs.app_rate_default_l_ha} unit="L/ha" />
          <SpecRow label="Мин. доза" value={specs.app_rate_min_l_ha} unit="L/ha" />
          <SpecRow label="Макс. доза" value={specs.app_rate_max_l_ha} unit="L/ha" />
          <SpecRow label="Рабочая высота" value={specs.spray_height_m} unit="m" />
        </Card>

        <Card title="Эксплуатация">
          <SpecRow label="Время обслуживания (точка встречи)" value={specs.service_time_s} unit="s" />
        </Card>

      </div>
    </div>
  )
}
