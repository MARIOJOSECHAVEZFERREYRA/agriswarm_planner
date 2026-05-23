import { useState, useMemo, useCallback, useEffect } from 'react'
import { lngLatToXy, xyToLngLat, setGeoOrigin, wouldSelfIntersect, polygonArea, pointInPolygon, isPolygonInside, polygonsOverlap } from '../utils/geo.js'
import { MODE } from '../utils/modes.js'

const MIN_FIELD_AREA_M2    = 100  // 10×10 m — below this the field is degenerate for planning
const MIN_OBSTACLE_AREA_M2 = 1    // 1 m²  — avoid pixel-size holes

const WARN = {
  SELF_INTERSECT:     'Invalid shape: edges cannot cross',
  DUPLICATE_POINT:    'Duplicate point: same location as previous',
  FIELD_TOO_SMALL:    `Field area too small (min ${MIN_FIELD_AREA_M2} m²)`,
  OBSTACLE_TOO_SMALL: `Obstacle area too small (min ${MIN_OBSTACLE_AREA_M2} m²)`,
  OBSTACLE_OUTSIDE:   'Obstacle must be fully inside the field boundary',
  OBSTACLE_OVERLAP:   'Obstacles cannot touch or overlap each other',
}

export function useFieldEditor(resetMission) {
  const [mode, setMode] = useState(MODE.NONE)
  const [drawingPoints, setDrawingPoints] = useState([])
  const [drawnPolygon, setDrawnPolygon] = useState(null)
  const [obstacles, setObstacles] = useState([])
  const [basePoint, setBasePoint] = useState(null)
  const [ugvRoute, setUgvRoute] = useState(null)
  const [intersectionWarning, setIntersectionWarning] = useState(null)
  // Metadata preserved from the loaded JSON so saving back keeps name,
  // description, geo-reference, UGV params, etc. that the editor does not
  // expose as editable state.
  const [loadedMeta, setLoadedMeta] = useState(null)
  // Optional File System Access handle: when present, saving writes back
  // to the same file on disk instead of offering a download / copy modal.
  const [fileHandle, setFileHandle] = useState(null)

  const activeField = useMemo(() => {
    if (drawnPolygon?.length >= 3) {
      return {
        coordinates: drawnPolygon,
        obstacles,
      }
    }
    return null
  }, [drawnPolygon, obstacles])

  useEffect(() => {
    let timeoutId = null

    if (intersectionWarning) {
      timeoutId = setTimeout(() => {
        setIntersectionWarning(null)
      }, 2500)
    }

    return () => {
      if (timeoutId !== null) {
        clearTimeout(timeoutId)
      }
    }
  }, [intersectionWarning])

  const resetEditorState = useCallback(() => {
    setDrawingPoints([])
    setMode(MODE.NONE)
    setIntersectionWarning(null)
  }, [])

  const resetDrawingAndMission = useCallback(() => {
    resetEditorState()
    resetMission()
  }, [resetEditorState, resetMission])

  const addPoint = useCallback((x, y) => {
    if (mode === MODE.NONE) return

    if (mode === MODE.SET_BASE_POINT) {
      setBasePoint([x, y])
      setMode(MODE.NONE)
      return
    }

    // UGV route is an open polyline — no intersection check needed
    if (mode === MODE.DRAW_UGV_ROUTE) {
      setDrawingPoints(points => [...points, [x, y]])
      return
    }

    // 1. Duplicate consecutive point
    if (drawingPoints.length > 0) {
      const [lx, ly] = drawingPoints[drawingPoints.length - 1]
      if (lx === x && ly === y) {
        setIntersectionWarning(WARN.DUPLICATE_POINT)
        return
      }
    }

    // 2. For obstacles: each new point must be inside the field boundary
    if (mode === MODE.DRAW_OBSTACLE && drawnPolygon && !pointInPolygon([x, y], drawnPolygon)) {
      setIntersectionWarning(WARN.OBSTACLE_OUTSIDE)
      return
    }

    // 3. Self-intersection check
    if (wouldSelfIntersect(drawingPoints, [x, y])) {
      setIntersectionWarning(WARN.SELF_INTERSECT)
      return
    }

    setDrawingPoints(points => [...points, [x, y]])
  }, [mode, drawingPoints, drawnPolygon])

  const undoPoint = useCallback(() => {
    if (mode !== MODE.NONE) {
      setDrawingPoints(points => points.slice(0, -1))
    }
  }, [mode])

  const handleMapClick = useCallback((lng, lat) => {
    const [x, y] = lngLatToXy(lng, lat)
    addPoint(x, y)
  }, [addPoint])

  const handleToggleDrawPolygon = useCallback(() => {
    if (mode === MODE.DRAW_POLYGON) {
      if (drawingPoints.length >= 3) {
        if (polygonArea(drawingPoints) < MIN_FIELD_AREA_M2) {
          setIntersectionWarning(WARN.FIELD_TOO_SMALL)
          return
        }
        // Recentre geo origin to polygon centroid so coords stay near (0,0)
        const n = drawingPoints.length
        const cx = drawingPoints.reduce((s, p) => s + p[0], 0) / n
        const cy = drawingPoints.reduce((s, p) => s + p[1], 0) / n
        const [cLng, cLat] = xyToLngLat(cx, cy)
        setGeoOrigin(cLng, cLat)
        // Reproject all points relative to new origin
        const centred = drawingPoints.map(([x, y]) => [x - cx, y - cy])
        setDrawnPolygon(centred)
      }
      setDrawingPoints([])
      setMode(MODE.NONE)
      setIntersectionWarning(null)
      return
    }

    setMode(MODE.DRAW_POLYGON)
    setDrawingPoints([])
    setIntersectionWarning(null)
  }, [mode, drawingPoints])

  const handleToggleDrawUgvRoute = useCallback(() => {
    if (mode === MODE.DRAW_UGV_ROUTE) {
      if (drawingPoints.length >= 2) {
        setUgvRoute([...drawingPoints])
      }
      setDrawingPoints([])
      setMode(MODE.NONE)
      return
    }
    setMode(MODE.DRAW_UGV_ROUTE)
    setDrawingPoints([])
  }, [mode, drawingPoints])

  const handleToggleSetBasePoint = useCallback(() => {
    setIntersectionWarning(null)
    setDrawingPoints([])
    setMode(currentMode =>
      currentMode === MODE.SET_BASE_POINT ? MODE.NONE : MODE.SET_BASE_POINT
    )
  }, [])

  const handleToggleDrawObstacle = useCallback(() => {
    if (mode === MODE.DRAW_OBSTACLE) {
      if (drawingPoints.length >= 3) {
        // Area check
        if (polygonArea(drawingPoints) < MIN_OBSTACLE_AREA_M2) {
          setIntersectionWarning(WARN.OBSTACLE_TOO_SMALL)
          setDrawingPoints([])
          setMode(MODE.NONE)
          return
        }
        // Containment check (obstacle must be fully inside field)
        if (drawnPolygon && !isPolygonInside(drawingPoints, drawnPolygon)) {
          setIntersectionWarning(WARN.OBSTACLE_OUTSIDE)
          setDrawingPoints([])
          setMode(MODE.NONE)
          return
        }
        // Overlap check against existing obstacles
        for (const obs of obstacles) {
          if (polygonsOverlap(drawingPoints, obs)) {
            setIntersectionWarning(WARN.OBSTACLE_OVERLAP)
            setDrawingPoints([])
            setMode(MODE.NONE)
            return
          }
        }
        setObstacles(currentObstacles => [...currentObstacles, [...drawingPoints]])
      }
      setDrawingPoints([])
      setMode(MODE.NONE)
      setIntersectionWarning(null)
      return
    }

    setMode(MODE.DRAW_OBSTACLE)
    setDrawingPoints([])
    setIntersectionWarning(null)
  }, [mode, drawingPoints, drawnPolygon, obstacles])

  const handleLoadField = useCallback((fieldData, handle = null) => {
    // Support both flat format { boundary, ugvRoute, ... } and
    // exported mission format { field: { boundary, ugvRoute, ... }, ... }.
    const src = fieldData.field ?? fieldData
    // If the file carries a real-world geographic origin (e.g. FTW
    // polygons normalized from UTM), anchor the local flat frame there
    // so the polygon renders at its true location on the map.
    const originLngLat = src._origin_lnglat ?? fieldData._origin_lnglat ?? null
    if (Array.isArray(originLngLat) && originLngLat.length === 2) {
      setGeoOrigin(originLngLat[0], originLngLat[1])
    }
    const route = src.ugvRoute ?? src.ugv_polyline ?? null
    setDrawnPolygon(src.boundary)
    setObstacles(src.obstacles ?? [])
    setBasePoint(src.basePoint ?? src.base_point ?? null)
    setUgvRoute(route)
    // Preserve metadata fields that the editor does not expose as state
    // so a later save round-trips them unchanged.
    setLoadedMeta({
      name: src.name ?? null,
      description: src.description ?? null,
      ugv_speed: src.ugv_speed ?? null,
      ugv_t_service: src.ugv_t_service ?? null,
      _origin_lnglat: src._origin_lnglat ?? null,
      _utm_origin: src._utm_origin ?? null,
      _utm_epsg: src._utm_epsg ?? null,
    })
    setFileHandle(handle)
    resetDrawingAndMission()
    // Return loaded route so caller can auto-switch mission type.
    return { ugvRoute: route }
  }, [resetDrawingAndMission])

  // Build a JSON-serializable field document from the current editor state
  // merged with the metadata preserved at load time.
  const buildFieldDoc = useCallback(() => {
    const doc = {}
    if (loadedMeta?.name) doc.name = loadedMeta.name
    if (loadedMeta?.description) doc.description = loadedMeta.description
    doc.boundary = drawnPolygon ?? []
    doc.obstacles = obstacles ?? []
    if (basePoint) doc.base_point = basePoint
    if (ugvRoute && ugvRoute.length >= 2) {
      doc.ugv_polyline = ugvRoute
      doc.ugv_speed = loadedMeta?.ugv_speed ?? 2.0
      doc.ugv_t_service = loadedMeta?.ugv_t_service ?? 300.0
    }
    if (loadedMeta?._utm_origin) doc._utm_origin = loadedMeta._utm_origin
    if (loadedMeta?._origin_lnglat) doc._origin_lnglat = loadedMeta._origin_lnglat
    if (loadedMeta?._utm_epsg) doc._utm_epsg = loadedMeta._utm_epsg
    return doc
  }, [loadedMeta, drawnPolygon, obstacles, basePoint, ugvRoute])

  const handleClear = useCallback(() => {
    setDrawnPolygon(null)
    setObstacles([])
    setBasePoint(null)
    setUgvRoute(null)
    setLoadedMeta(null)
    setFileHandle(null)
    resetDrawingAndMission()
  }, [resetDrawingAndMission])

  return {
    mode,
    drawingPoints,
    activeField,
    basePoint,
    ugvRoute,
    intersectionWarning,
    loadedMeta,
    fileHandle,
    addPoint,
    undoPoint,
    handleMapClick,
    handleToggleDrawPolygon,
    handleToggleSetBasePoint,
    handleToggleDrawObstacle,
    handleToggleDrawUgvRoute,
    handleLoadField,
    handleClear,
    setBasePoint,
    setUgvRoute,
    buildFieldDoc,
  }
}