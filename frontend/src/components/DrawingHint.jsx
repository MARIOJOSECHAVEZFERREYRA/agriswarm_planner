import FloatingPanel from './FloatingPanel.jsx'

const DRAWING_HINT_STYLE = {
  bottom: 24,
  left: '50%',
  transform: 'translateX(-50%)',
  padding: '8px 18px',
  fontSize: 13,
  zIndex: 10,
}

export default function DrawingHint({ mode }) {
  let text = ''

  if (mode === 'draw_polygon') {
    text = 'Щёлкните, чтобы добавить точки · Правый клик — отменить · По завершении нажмите «Готово»'
  } else if (mode === 'set_base_point') {
    text = 'Щёлкните на карте, чтобы задать базу или точку подзарядки'
  } else if (mode === 'draw_ugv_route') {
    text = 'Щёлкните, чтобы добавить путевые точки НТС · Правый клик — отменить · По завершении нажмите «Завершить маршрут НТС» (мин. 2 точки)'
  } else {
    text = 'Щёлкните, чтобы добавить точки препятствия · Правый клик — отменить · По завершении нажмите «Завершить препятствие»'
  }

  return (
    <FloatingPanel tone="accent" style={DRAWING_HINT_STYLE}>
      {text}
    </FloatingPanel>
  )
}
