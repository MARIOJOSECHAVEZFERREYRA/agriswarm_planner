import { useCallback, useRef, useState } from 'react'
import { ensureClosed, polylineLength, formatMeters } from '../utils/geo.js'
import { MODE } from '../utils/modes.js'
import { C } from '../utils/colors.js'
import { useDroneCatalog } from '../hooks/useDroneCatalog.js'
import { useMissionCompute } from '../hooks/useMissionCompute.js'

const STATUS_COLOR = {
  pending:   C.warning,
  running:   C.accent,
  completed: C.success,
  failed:    C.error,
}

const STATUS_LABEL = {
  pending:   'ожидание',
  running:   'выполняется',
  completed: 'завершено',
  failed:    'ошибка',
}

const s = {
  panel: {
    display: 'flex', flexDirection: 'column', gap: 0,
    background: C.bg, height: '100%', overflowY: 'auto',
    fontFamily: "'Inter', 'Segoe UI', system-ui, sans-serif",
  },
  header: {
    padding: '18px 16px 14px',
    borderBottom: `1px solid ${C.border}`,
  },
  logo: { display: 'flex', alignItems: 'center', gap: 8, marginBottom: 2 },
  logoIcon: { fontSize: 18 },
  logoText: { fontSize: 15, fontWeight: 700, color: C.text, letterSpacing: 0.3 },
  subtitle: { fontSize: 11, color: C.muted, letterSpacing: 0.2 },
  section: {
    padding: '14px 16px',
    borderBottom: `1px solid ${C.border}`,
    display: 'flex', flexDirection: 'column', gap: 10,
  },
  sectionLabel: {
    fontSize: 10, fontWeight: 700, color: C.muted,
    letterSpacing: 1.8, textTransform: 'uppercase',
  },
  label: { fontSize: 11, color: C.muted, marginBottom: 3 },
  input: {
    width: '100%', padding: '7px 10px', boxSizing: 'border-box',
    background: C.surface, border: `1px solid ${C.border}`,
    borderRadius: 6, color: C.text, fontSize: 12,
    outline: 'none',
  },
  select: {
    width: '100%', padding: '7px 10px',
    background: C.surface, border: `1px solid ${C.border}`,
    borderRadius: 6, color: C.text, fontSize: 12,
    outline: 'none',
  },
  btnRow: { display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 8 },
  stat: { display: 'flex', justifyContent: 'space-between', alignItems: 'center', fontSize: 12 },
  statKey: { color: C.muted },
  statVal: { color: C.text, fontWeight: 600, fontVariantNumeric: 'tabular-nums' },
  badge: {
    display: 'inline-flex', alignItems: 'center', gap: 5,
    padding: '2px 9px', borderRadius: 20, fontSize: 11, fontWeight: 600,
  },
  errorBox: {
    background: `${C.error}12`, border: `1px solid ${C.error}`,
    borderRadius: 6, padding: '8px 10px', fontSize: 11, color: '#ffa198',
  },
}

function Btn({ children, onClick, variant = 'default', active = false, disabled = false, fullWidth = true }) {
  const variants = {
    default:  { bg: C.surface,   border: C.border,    color: C.text },
    primary:  { bg: C.accentDim, border: C.accent,    color: '#ffffff' },
    success:  { bg: `${C.success}18`, border: `${C.success}80`, color: C.success },
    danger:   { bg: `${C.danger}25`,  border: C.danger,         color: '#ff7b72' },
    active:   { bg: `${C.accent}28`,  border: C.accent,         color: C.accent },
    warning:  { bg: `${C.warning}28`, border: `${C.warning}90`, color: '#e3b341' },
  }
  const v = active ? variants.active : variants[variant]
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      style={{
        display: 'flex', alignItems: 'center', justifyContent: 'center', gap: 6,
        padding: '8px 12px', border: `1px solid ${v.border}`, borderRadius: 6,
        background: v.bg, color: v.color,
        fontSize: 12, fontWeight: 500, cursor: disabled ? 'not-allowed' : 'pointer',
        opacity: disabled ? 0.45 : 1, width: fullWidth ? '100%' : 'auto',
        transition: 'opacity 0.15s, background 0.15s',
        fontFamily: 'inherit',
      }}
    >
      {children}
    </button>
  )
}

function Divider() {
  return <div style={{ height: 1, background: C.border }} />
}

function Spinner() {
  return (
    <span style={{
      display: 'inline-block', width: 11, height: 11,
      border: '2px solid #ffffff40', borderTopColor: '#ffffff',
      borderRadius: '50%', animation: 'spin 0.7s linear infinite',
    }} />
  )
}

function HeaderSection() {
  return (
    <div style={s.header}>
      <div style={s.logo}>
        <span style={s.logoText}>Планировщик миссий БПЛА-НТС</span>
      </div>
    </div>
  )
}

function AircraftSection({ drones, drone, onDroneChange, onViewSpecs }) {
  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>БПЛА</div>
      <div>
        <div style={s.label}>Модель БПЛА</div>
        <div style={{ display: 'flex', gap: 6 }}>
          <select style={{ ...s.select, flex: 1 }} value={drone} onChange={e => onDroneChange(e.target.value)}>
            {drones.map(d => <option key={d.name} value={d.name}>{d.name}</option>)}
          </select>
          <button
            onClick={() => drone && onViewSpecs(drone)}
            disabled={!drone}
            title="Просмотр характеристик БПЛА"
            style={{
              flexShrink: 0,
              width: 32, height: 32,
              display: 'flex', alignItems: 'center', justifyContent: 'center',
              background: C.surface, border: `1px solid ${C.border}`,
              borderRadius: 6, color: C.accent,
              fontSize: 13, fontWeight: 700,
              cursor: drone ? 'pointer' : 'not-allowed',
              opacity: drone ? 1 : 0.4,
              fontFamily: 'inherit',
            }}
          >
            i
          </button>
        </div>
      </div>
    </div>
  )
}

function FieldSection({
  mode, activeField, drawingPtsCount,
  onToggleDrawPolygon, onToggleDrawObstacle,
  onLoadField, onClear,
  fileHandle, loadedMetaName, buildFieldDoc,
}) {
  const fileRef = useRef(null)
  const [jsonModal, setJsonModal] = useState(null)
  const [toast, setToast] = useState(null)
  const isDrawingPolygon  = mode === MODE.DRAW_POLYGON
  const isDrawingObstacle = mode === MODE.DRAW_OBSTACLE
  const hasField          = !!activeField
  const fsaSupported      = typeof window !== 'undefined' && 'showOpenFilePicker' in window

  function handleLoadJSON(e) {
    const file = e.target.files[0]
    if (!file) return
    const reader = new FileReader()
    reader.onload = (ev) => {
      try { onLoadField(JSON.parse(ev.target.result)) }
      catch { alert('Неверный файл JSON') }
    }
    reader.readAsText(file)
    e.target.value = ''
  }

  async function handleOpenFile() {
    // File System Access API path: pick a file AND keep the handle so we
    // can write back in-place on save.
    try {
      const [handle] = await window.showOpenFilePicker({
        types: [{ description: 'JSON поля', accept: { 'application/json': ['.json'] } }],
      })
      const file = await handle.getFile()
      const text = await file.text()
      onLoadField(JSON.parse(text), handle)
    } catch (err) {
      if (err?.name !== 'AbortError') alert('Не удалось открыть файл: ' + err.message)
    }
  }

  async function handleSaveField() {
    const doc = buildFieldDoc()
    if (!doc.boundary || doc.boundary.length < 3) {
      alert('Сначала нарисуйте или загрузите поле.')
      return
    }
    const json = JSON.stringify(doc, null, 2)

    if (fileHandle) {
      try {
        const writable = await fileHandle.createWritable()
        await writable.write(json)
        await writable.close()
        setToast(`Сохранено в ${fileHandle.name}`)
        setTimeout(() => setToast(null), 2000)
        return
      } catch (err) {
        alert('Ошибка записи: ' + err.message)
        return
      }
    }
    setJsonModal(json)
  }

  async function handleCopyJson() {
    try {
      await navigator.clipboard.writeText(jsonModal)
      setToast('Скопировано в буфер обмена')
      setTimeout(() => setToast(null), 1500)
    } catch {
      // clipboard write may be blocked; the textarea is still selectable.
    }
  }

  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Поле</div>

      <div style={s.btnRow}>
        <Btn
          variant={isDrawingPolygon ? 'active' : 'success'}
          active={isDrawingPolygon}
          onClick={onToggleDrawPolygon}
        >
          {isDrawingPolygon ? `Завершить (${drawingPtsCount} тчк)` : 'Нарисовать поле'}
        </Btn>
        <Btn
          variant={isDrawingObstacle ? 'warning' : 'default'}
          active={isDrawingObstacle}
          disabled={!hasField && !isDrawingObstacle}
          onClick={onToggleDrawObstacle}
        >
          {isDrawingObstacle ? 'Завершить препятствие' : 'Добавить препятствие'}
        </Btn>
      </div>

      <div style={s.btnRow}>
        {fsaSupported
          ? <Btn variant='default' onClick={handleOpenFile}>Открыть поле</Btn>
          : <Btn variant='default' onClick={() => fileRef.current.click()}>Загрузить JSON</Btn>}
        <Btn variant='default' disabled={!hasField} onClick={handleSaveField}>
          {fileHandle ? 'Сохранить' : 'Сохранить как…'}
        </Btn>
      </div>

      <div style={s.btnRow}>
        <Btn variant='danger' disabled={!hasField && mode === MODE.NONE} onClick={onClear}>Очистить всё</Btn>
      </div>

      <input ref={fileRef} type="file" accept=".json" style={{ display: 'none' }} onChange={handleLoadJSON} />

      {fileHandle && (
        <div style={{ fontSize: 11, color: C.muted }}>
          Файл: <span style={{ color: '#9ad0ff' }}>{fileHandle.name}</span>
          {loadedMetaName ? ` (${loadedMetaName})` : ''}
        </div>
      )}

      {activeField?.obstacles?.length > 0 && (
        <div style={{ fontSize: 11, color: C.muted, display: 'flex', alignItems: 'center', gap: 5 }}>
          <span style={{ color: '#e36d2e' }}>+</span>
          Задано препятствий: {activeField.obstacles.length}
        </div>
      )}

      {jsonModal && (
        <div style={{
          position: 'fixed', inset: 0, background: 'rgba(0,0,0,0.65)',
          zIndex: 9999, display: 'flex', alignItems: 'center', justifyContent: 'center',
        }} onClick={() => setJsonModal(null)}>
          <div style={{
            background: '#111', padding: 16, borderRadius: 8, width: '80%',
            maxWidth: 900, maxHeight: '80vh', display: 'flex', flexDirection: 'column',
            gap: 10, border: `1px solid ${C.border}`,
          }} onClick={e => e.stopPropagation()}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <div style={{ fontSize: 13, color: C.muted }}>
                Скопируйте этот JSON и сохраните его в файл вашего поля.
              </div>
              <div style={{ display: 'flex', gap: 8 }}>
                <Btn variant='default' onClick={handleCopyJson}>Копировать</Btn>
                <Btn variant='danger' onClick={() => setJsonModal(null)}>Закрыть</Btn>
              </div>
            </div>
            <textarea
              readOnly
              value={jsonModal}
              style={{
                flex: 1, minHeight: 300, fontFamily: 'monospace', fontSize: 12,
                padding: 8, background: '#000', color: '#ddd', border: `1px solid ${C.border}`,
                borderRadius: 4, resize: 'vertical',
              }}
              onFocus={e => e.target.select()}
            />
          </div>
        </div>
      )}

      {toast && (
        <div style={{
          position: 'fixed', bottom: 20, left: '50%', transform: 'translateX(-50%)',
          background: '#1f6feb', color: 'white', padding: '8px 14px', borderRadius: 6,
          fontSize: 13, zIndex: 10000, boxShadow: '0 4px 12px rgba(0,0,0,0.4)',
        }}>{toast}</div>
      )}
    </div>
  )
}

const MISSION_TYPES = [
  {
    value: 'static',
    label: 'Стационарная база',
    description: 'БПЛА возвращается на стационарную базовую станцию для подзарядки и дозаправки.',
  },
  {
    value: 'mobile',
    label: 'Подвижный НТС',
    description: 'НТС движется по маршруту; БПЛА встречает его в точках встречи.',
  },
]

function GroundSection({
  missionType, onMissionTypeChange,
  mode, basePoint, onToggleSetBasePoint,
  ugvRoute, drawingPtsCount, drawingLengthM,
  ugvSpeed, setUgvSpeed, ugvTService, setUgvTService,
  onToggleDrawUgvRoute,
}) {
  const isSettingBase = mode === MODE.SET_BASE_POINT
  const isDrawingUgv  = mode === MODE.DRAW_UGV_ROUTE
  const hasRoute      = ugvRoute && ugvRoute.length >= 2

  const routeLengthM = hasRoute ? polylineLength(ugvRoute) : 0

  const isMobile = missionType === 'mobile'
  const typeInfo  = MISSION_TYPES.find(t => t.value === missionType)

  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Наземные операции</div>

      {/* Mode toggle */}
      <div style={{
        display: 'grid', gridTemplateColumns: '1fr 1fr',
        borderRadius: 7, overflow: 'hidden',
        border: `1px solid ${C.border}`,
      }}>
        {MISSION_TYPES.map((opt, i) => {
          const active = missionType === opt.value
          return (
            <button
              key={opt.value}
              onClick={() => onMissionTypeChange(opt.value)}
              style={{
                padding: '10px 8px',
                background: active ? C.accentDim : C.surface,
                border: 'none',
                borderLeft: i > 0 ? `1px solid ${C.border}` : 'none',
                color: active ? '#fff' : C.muted,
                fontSize: 11, fontWeight: active ? 700 : 500,
                cursor: 'pointer', fontFamily: 'inherit',
                transition: 'background 0.15s, color 0.15s',
                lineHeight: 1.3,
              }}
            >
              {opt.label}
            </button>
          )
        })}
      </div>

      <div style={{ fontSize: 11, color: C.muted, lineHeight: 1.5 }}>
        {typeInfo?.description}
      </div>

      <Divider />

      {/* Base station (static only — mobile derives start from polyline) */}
      {!isMobile && (
        <>
          <Btn
            variant={isSettingBase ? 'active' : 'default'}
            active={isSettingBase}
            onClick={onToggleSetBasePoint}
          >
            {isSettingBase
              ? 'Нажмите на карте…'
              : basePoint
                ? 'Переместить базовую станцию'
                : 'Задать базовую станцию'}
          </Btn>

          {basePoint && (
            <div style={{ fontSize: 11, color: C.muted }}>
              База: ({basePoint[0].toFixed(1)}, {basePoint[1].toFixed(1)})
            </div>
          )}
        </>
      )}

      {/* Mobile-only UGV route controls */}
      {isMobile && (
        <>
          <Divider />

          <Btn
            variant={isDrawingUgv ? 'active' : hasRoute ? 'warning' : 'default'}
            active={isDrawingUgv}
            onClick={onToggleDrawUgvRoute}
          >
            {isDrawingUgv
              ? drawingLengthM > 0
                ? `Завершить маршрут (${formatMeters(drawingLengthM)})`
                : `Завершить маршрут (${drawingPtsCount} тчк)`
              : hasRoute
                ? `Перерисовать маршрут НТС (${formatMeters(routeLengthM)})`
                : 'Нарисовать маршрут НТС'}
          </Btn>

          {hasRoute && !isDrawingUgv && (
            <div style={{ fontSize: 11, color: C.muted }}>
              Маршрут: {formatMeters(routeLengthM)} · {ugvRoute.length} путевых точек
            </div>
          )}

          <div style={s.btnRow}>
            <div>
              <div style={s.label}>Скорость НТС (м/с)</div>
              <input
                style={s.input} type="number"
                min={0.1} max={10} step={0.1}
                value={ugvSpeed} onChange={e => setUgvSpeed(e.target.value)}
              />
            </div>
            <div>
              <div style={s.label}>Время обслуживания (с)</div>
              <input
                style={s.input} type="number"
                min={30} max={1800} step={30}
                value={ugvTService} onChange={e => setUgvTService(e.target.value)}
              />
            </div>
          </div>
        </>
      )}
    </div>
  )
}

function RangeHint({ min, max, unit = '' }) {
  if (min == null || max == null) return null
  return (
    <span style={{ color: C.muted, fontSize: 10 }}>
      {min}{unit}–{max}{unit}
    </span>
  )
}

function ParamInput({ label, value, onChange, min, max, step = 0.5, hint }) {
  return (
    <div>
      <div style={{ ...s.label, display: 'flex', justifyContent: 'space-between' }}>
        <span>{label}</span>
        {hint}
      </div>
      <input
        style={s.input} type="number"
        min={min} max={max} step={step}
        value={value} onChange={e => onChange(e.target.value)}
      />
    </div>
  )
}

function ParametersSection({
  sprayWidth, setSprayWidth,
  appRate, setAppRate,
  speed, setSpeed,
  margin, setMargin,
  defaults,
  strategy, setStrategy,
  fixedAngle, setFixedAngle,
}) {
  const d = defaults

  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Параметры миссии</div>

      <div style={s.btnRow}>
        <ParamInput
          label="Ширина распыления (м)"
          value={sprayWidth} onChange={setSprayWidth}
          min={d?.spray_swath_min_m} max={d?.spray_swath_max_m}
          hint={<RangeHint min={d?.spray_swath_min_m} max={d?.spray_swath_max_m} unit=" m" />}
        />
        <ParamInput
          label="Норма внесения (л/га)"
          value={appRate} onChange={setAppRate}
          min={d?.app_rate_min_l_ha} max={d?.app_rate_max_l_ha}
          hint={<RangeHint min={d?.app_rate_min_l_ha} max={d?.app_rate_max_l_ha} />}
        />
      </div>

      <div style={s.btnRow}>
        <ParamInput
          label="Скорость (м/с)"
          value={speed} onChange={setSpeed}
          min={d?.speed_min_ms} max={d?.speed_max_ms}
          hint={<RangeHint min={d?.speed_min_ms} max={d?.speed_max_ms} unit=" m/s" />}
        />
        <ParamInput
          label="Отступ (м)"
          value={margin} onChange={setMargin}
          min={0}
        />
      </div>

      <div>
        <div style={s.label}>Стратегия планирования</div>
        <select style={s.select} value={strategy} onChange={e => setStrategy(e.target.value)}>
          <option value="grid">GA (исчерпывающий, рекомендуется)</option>
          <option value="genetic">Генетический алгоритм (GA)</option>
          <option value="fixed">Фиксированный угол (без оптимизации)</option>
        </select>
      </div>

      {strategy === 'fixed' && (
        <div style={{ marginTop: 8 }}>
          <ParamInput
            label="Угол прохода (°)"
            value={fixedAngle}
            onChange={setFixedAngle}
            min={0}
            max={179}
            hint={<RangeHint min={0} max={179} unit="°" />}
          />
        </div>
      )}
    </div>
  )
}

function MetricRow({ label, value }) {
  return (
    <div style={s.stat}>
      <span style={s.statKey}>{label}</span>
      <span style={s.statVal}>{value}</span>
    </div>
  )
}

function ResultsSection({ mission, onExport, onStartSim, onStopSim, simEnabled }) {
  const metrics = (() => {
    try { return mission.metrics_json ? JSON.parse(mission.metrics_json) : null }
    catch { return null }
  })()

  return (
    <div style={s.section}>
      <div style={s.sectionLabel}>Результаты</div>

      <div style={s.stat}>
        <span style={s.statKey}>Статус</span>
        <span style={{
          ...s.badge,
          background: STATUS_COLOR[mission.status] + '20',
          color: STATUS_COLOR[mission.status],
          border: `1px solid ${STATUS_COLOR[mission.status]}40`,
        }}>
          {STATUS_LABEL[mission.status] ?? mission.status}
        </span>
      </div>

      {mission.status === 'completed' && <>
        <Divider />

        {/* Geometry */}
        <MetricRow
          label="Оптимальный угол"
          value={`${mission.best_angle?.toFixed(1)}°`}
        />
        {mission.coverage_area && (
          <MetricRow
            label="Площадь покрытия"
            value={`${(mission.coverage_area / 10000).toFixed(2)} га`}
          />
        )}
        {mission.n_cycles != null && (
          <MetricRow label="Циклы полёта" value={mission.n_cycles} />
        )}
        <MetricRow label="Путевые точки" value={mission.waypoints?.length} />

        {/* Mission metrics from MissionAnalyzer */}
        {metrics && <>
          <Divider />
          <MetricRow
            label="Дистанция распыления"
            value={`${metrics.spray_dist_km?.toFixed(3)} км`}
          />
          <MetricRow
            label="Холостая дистанция"
            value={`${metrics.dead_dist_km?.toFixed(3)} км`}
          />
          <MetricRow
            label="Эффективность"
            value={`${metrics.efficiency_ratio?.toFixed(1)} %`}
          />
          <MetricRow
            label="Время полёта"
            value={`${metrics.flight_time_min?.toFixed(1)} мин`}
          />
          <MetricRow
            label="Общее время операции"
            value={`${metrics.total_op_time_min?.toFixed(1)} мин`}
          />
          <MetricRow
            label="Производительность"
            value={`${metrics.productivity_ha_hr?.toFixed(2)} га/ч`}
          />
          <MetricRow
            label="Внесённая доза"
            value={`${metrics.real_dosage_l_ha?.toFixed(1)} л/га`}
          />
        </>}

        {metrics?.rv_n_rendezvous != null && metrics.rv_n_rendezvous > 0 && <>
          <Divider />
          <MetricRow label="Остановки в точках встречи" value={metrics.rv_n_rendezvous} />
          <MetricRow
            label="Ожидание БПЛА (всего)"
            value={`${metrics.rv_wait_min?.toFixed(1)} мин`}
          />
        </>}

        <Divider />
        {simEnabled ? (
          <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
            <span style={{
              flex: 1, fontSize: 11, color: C.success, fontWeight: 600,
              display: 'flex', alignItems: 'center', gap: 5,
            }}>
              <span style={{
                width: 7, height: 7, borderRadius: '50%',
                background: C.success, display: 'inline-block',
                boxShadow: `0 0 5px ${C.success}`,
              }} />
              Симуляция активна
            </span>
            <Btn variant='danger' fullWidth={false} onClick={onStopSim}>Стоп</Btn>
          </div>
        ) : (
          <Btn variant='primary' onClick={onStartSim}>Запустить симуляцию</Btn>
        )}
        <Btn variant='default' onClick={onExport}>Экспорт миссии</Btn>
      </>}

      {mission.status === 'failed' && (
        <div style={s.errorBox}>{mission.error_message}</div>
      )}
    </div>
  )
}

function ExportDialog({ onConfirm, onCancel }) {
  const [name, setName] = useState('Миссия')
  return (
    <div style={{
      position: 'fixed', inset: 0, zIndex: 1000,
      background: 'rgba(0,0,0,0.55)',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    }}>
      <div style={{
        background: C.bg, border: `1px solid ${C.border}`,
        borderRadius: 10, padding: '20px 22px', width: 300,
        display: 'flex', flexDirection: 'column', gap: 14,
        boxShadow: '0 8px 32px rgba(0,0,0,0.4)',
      }}>
        <div style={{ fontSize: 13, fontWeight: 700, color: C.text }}>Экспорт миссии</div>
        <div>
          <div style={s.label}>Имя файла</div>
          <input
            style={s.input}
            value={name}
            onChange={e => setName(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && name.trim() && onConfirm(name.trim())}
            autoFocus
          />
        </div>
        <div style={{ display: 'flex', gap: 8 }}>
          <Btn variant='default' onClick={onCancel}>Отмена</Btn>
          <Btn variant='primary' disabled={!name.trim()} onClick={() => onConfirm(name.trim())}>
            Скачать JSON
          </Btn>
        </div>
      </div>
    </div>
  )
}

export default function MissionPanel({
  mode, activeField, drawingPtsCount, drawingLengthM, basePoint,
  ugvRoute,
  onToggleDrawPolygon, onToggleDrawObstacle, onToggleSetBasePoint,
  onToggleDrawUgvRoute,
  onLoadField, onClear, onMissionReady, onStartSim, onStopSim, simEnabled, onViewDroneSpecs,
  setBasePoint, setUgvRoute, resetMission,
  fileHandle, loadedMetaName, buildFieldDoc,
}) {
  const [validationError, setValidationError] = useState(null)
  const [missionType, setMissionTypeRaw] = useState('static')
  const [sprayWidth, setSprayWidth] = useState('')
  const [speed, setSpeed] = useState('')
  const [appRate, setAppRate] = useState('')
  const [margin, setMargin] = useState('')
  const [strategy, setStrategy] = useState('grid')
  const [fixedAngle, setFixedAngle] = useState(0)
  const [ugvSpeed, setUgvSpeed] = useState(2.0)
  const [ugvTService, setUgvTService] = useState(300)
  const [showExport, setShowExport] = useState(false)

  const applyDroneDefaults = useCallback((nextDefaults) => {
    setSprayWidth(nextDefaults.swath_m)
    setSpeed(nextDefaults.speed_ms)
    setAppRate(nextDefaults.app_rate_l_ha)
    setMargin(nextDefaults.margin_m)
  }, [])

  const {
    drones,
    drone,
    defaults,
    error: droneError,
    selectDrone,
  } = useDroneCatalog(applyDroneDefaults)

  const {
    mission,
    loading,
    error: missionError,
    computeMission,
    resetMissionState,
  } = useMissionCompute(onMissionReady)

  const setMissionType = useCallback((type) => {
    setMissionTypeRaw(type)
    setBasePoint(null)
    setUgvRoute(null)
    resetMission()
    resetMissionState()
    setValidationError(null)
  }, [resetMission, resetMissionState, setBasePoint, setUgvRoute])

  function handleDroneChange(name) {
    setValidationError(null)
    void selectDrone(name)
  }

  async function handleCompute() {
    if (!activeField) return
    const isMobileMode = missionType === 'mobile'
    const hasUgvRouteOk = ugvRoute && ugvRoute.length >= 2
    if (isMobileMode && !hasUgvRouteOk) {
      setValidationError('Пожалуйста, нарисуйте маршрут НТС перед расчётом миссии.')
      return
    }
    if (!isMobileMode && !basePoint) {
      setValidationError('Пожалуйста, задайте точку базы перед расчётом миссии.')
      return
    }
    const sw = Number(sprayWidth)
    const ar = Number(appRate)
    const sp = Number(speed)
    const mg = Number(margin)

    const rangeCheck = (val, name, unit, minKey, maxKey) => {
      if (!val || val <= 0) return `${name}: значение должно быть > 0.`
      if (defaults?.[minKey] != null && val < defaults[minKey]) return `${name}: слишком мало (мин ${defaults[minKey]} ${unit}).`
      if (defaults?.[maxKey] != null && val > defaults[maxKey]) return `${name}: слишком много (макс ${defaults[maxKey]} ${unit}).`
      return null
    }

    const err =
      rangeCheck(sw, 'Ширина распыления', 'м', 'spray_swath_min_m', 'spray_swath_max_m') ||
      rangeCheck(ar, 'Норма внесения', 'л/га', 'app_rate_min_l_ha', 'app_rate_max_l_ha') ||
      rangeCheck(sp, 'Скорость', 'м/с', 'speed_min_ms', 'speed_max_ms') ||
      (isNaN(mg) || mg < 0 ? 'Отступ должен быть >= 0.' : null)
    if (err) {
      setValidationError(err)
      return
    }

    setValidationError(null)

    const coords = ensureClosed(activeField.coordinates)
    const obs = (activeField.obstacles ?? []).map(ensureClosed)
    const isMobile = missionType === 'mobile'
    const hasUgvRoute = ugvRoute && ugvRoute.length >= 2
    const fieldPayload = {
      coordinates: coords,
      obstacles: obs,
      base_point: isMobile && hasUgvRoute ? ugvRoute[0] : basePoint,
    }

    if (isMobile && hasUgvRoute) {
      fieldPayload.ugv_polyline = ugvRoute
      fieldPayload.ugv_speed = Number(ugvSpeed)
      fieldPayload.ugv_t_service = Number(ugvTService)
    }

    const strategyParams = strategy === 'fixed'
      ? { angle: Number(fixedAngle) }
      : undefined

    await computeMission({
      name: `Mission ${new Date().toISOString().slice(0, 16).replace('T', ' ')}`,
      field: fieldPayload,
      spray_width: sw,
      strategy,
      strategy_params: strategyParams,
      drone_name: drone,
      app_rate: ar,
      cruise_speed_ms: sp,
      margin_m: mg,
    })
  }

  function handleExport(name) {
    if (!mission?.waypoints?.length) return
    const metrics = (() => {
      try { return mission.metrics_json ? JSON.parse(mission.metrics_json) : {} }
      catch { return {} }
    })()
    const data = {
      mission_name: name, drone,
      strategy: mission.strategy,
      best_angle_deg: mission.best_angle,
      n_cycles: mission.n_cycles,
      total_distance_m: mission.total_distance,
      coverage_area_m2: mission.coverage_area,
      spray_width_m: mission.spray_width,
      overrides: {
        swath: parseFloat(sprayWidth) || undefined,
        speed: parseFloat(speed) || undefined,
        app_rate: parseFloat(appRate) || undefined,
        margin: parseFloat(margin) || undefined,
      },
      field: activeField ? {
        boundary: activeField.coordinates,
        obstacles: activeField.obstacles ?? [],
        basePoint: basePoint ?? undefined,
        ugvRoute: ugvRoute ?? undefined,
      } : undefined,
      metrics,
      waypoints: mission.waypoints.map(w => ({
        sequence: w.sequence, x: w.x, y: w.y, type: w.waypoint_type,
        cycle_index: w.cycle_index,
      })),
    }
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' })
    const url  = URL.createObjectURL(blob)
    const a    = document.createElement('a')
    a.href = url
    a.download = `${name.replace(/\s+/g, '_')}.json`
    a.click()
    URL.revokeObjectURL(url)
    setShowExport(false)
  }

  const hasField      = !!activeField
  const hasBasePoint  = !!basePoint
  const needsUgvRoute = missionType === 'mobile'
  const hasUgvRoute   = ugvRoute && ugvRoute.length >= 2
  const groundReady   = needsUgvRoute ? hasUgvRoute : hasBasePoint
  const canCompute    = hasField && groundReady && !loading && mode === MODE.NONE
  const error = validationError ?? missionError ?? droneError

  const handleFieldLoad = useCallback((data, handle) => {
    resetMissionState()
    setValidationError(null)
    const result = onLoadField(data, handle)
    setMissionTypeRaw(result?.ugvRoute?.length >= 2 ? 'mobile' : 'static')
  }, [onLoadField, resetMissionState])

  const handleClearAll = useCallback(() => {
    resetMissionState()
    setValidationError(null)
    onClear()
  }, [onClear, resetMissionState])

  return (
    <div style={s.panel}>
      <HeaderSection />

      <AircraftSection
        drones={drones}
        drone={drone}
        onDroneChange={handleDroneChange}
        onViewSpecs={onViewDroneSpecs}
      />

      <FieldSection
        mode={mode}
        activeField={activeField}
        drawingPtsCount={drawingPtsCount}
        onToggleDrawPolygon={onToggleDrawPolygon}
        onToggleDrawObstacle={onToggleDrawObstacle}
        onLoadField={handleFieldLoad}
        onClear={handleClearAll}
        fileHandle={fileHandle}
        loadedMetaName={loadedMetaName}
        buildFieldDoc={buildFieldDoc}
      />

      <GroundSection
        missionType={missionType}
        onMissionTypeChange={setMissionType}
        mode={mode}
        basePoint={basePoint}
        onToggleSetBasePoint={onToggleSetBasePoint}
        ugvRoute={ugvRoute}
        drawingPtsCount={drawingPtsCount}
        drawingLengthM={drawingLengthM}
        ugvSpeed={ugvSpeed}
        setUgvSpeed={setUgvSpeed}
        ugvTService={ugvTService}
        setUgvTService={setUgvTService}
        onToggleDrawUgvRoute={onToggleDrawUgvRoute}
      />

      <ParametersSection
        sprayWidth={sprayWidth}
        setSprayWidth={setSprayWidth}
        appRate={appRate}
        setAppRate={setAppRate}
        speed={speed}
        setSpeed={setSpeed}
        margin={margin}
        setMargin={setMargin}
        defaults={defaults}
        strategy={strategy}
        setStrategy={setStrategy}
        fixedAngle={fixedAngle}
        setFixedAngle={setFixedAngle}
      />

      <div style={{ padding: '14px 16px', borderBottom: `1px solid ${C.border}` }}>
        {error && <div style={{ ...s.errorBox, marginBottom: 10 }}>{error}</div>}
        <Btn variant='primary' disabled={!canCompute} onClick={handleCompute}>
          {loading
            ? <><Spinner /> Расчёт…</>
            : 'Рассчитать миссию'}
        </Btn>
        {mode === MODE.DRAW_UGV_ROUTE && (
          <div style={{ fontSize: 11, color: C.warning, textAlign: 'center', marginTop: 7 }}>
            Завершите маршрут НТС перед расчётом
          </div>
        )}
        {mode !== MODE.DRAW_UGV_ROUTE && !hasField && (
          <div style={{ fontSize: 11, color: C.muted, textAlign: 'center', marginTop: 7 }}>
            Нарисуйте или загрузите поле, чтобы продолжить
          </div>
        )}
        {mode !== MODE.DRAW_UGV_ROUTE && hasField && !needsUgvRoute && !hasBasePoint && (
          <div style={{ fontSize: 11, color: C.warning, textAlign: 'center', marginTop: 7 }}>
            Задайте базовую станцию перед расчётом
          </div>
        )}
        {mode !== MODE.DRAW_UGV_ROUTE && hasField && needsUgvRoute && !hasUgvRoute && (
          <div style={{ fontSize: 11, color: C.warning, textAlign: 'center', marginTop: 7 }}>
            Нарисуйте маршрут НТС, чтобы включить подвижную точку встречи
          </div>
        )}
      </div>

      {mission && (
        <ResultsSection
          mission={mission}
          onExport={() => setShowExport(true)}
          onStartSim={onStartSim}
          onStopSim={onStopSim}
          simEnabled={simEnabled}
        />
      )}

      {showExport && (
        <ExportDialog
          onConfirm={handleExport}
          onCancel={() => setShowExport(false)}
        />
      )}
    </div>
  )
}
