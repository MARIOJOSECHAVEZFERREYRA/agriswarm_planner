import { useCallback, useEffect, useState } from 'react'

function toErrorMessage(error, fallback) {
  if (error instanceof Error && error.message) {
    return error.message
  }
  return fallback
}

export function useDroneCatalog(onDefaultsLoaded) {
  const [drones, setDrones] = useState([])
  const [drone, setDrone] = useState('')
  const [defaults, setDefaults] = useState(null)
  const [error, setError] = useState(null)

  const loadDroneDefaults = useCallback(async (name, signal) => {
    const response = await fetch(`/drones/${encodeURIComponent(name)}/defaults`, { signal })
    if (!response.ok) {
      throw new Error('Не удалось загрузить параметры БПЛА по умолчанию')
    }

    const nextDefaults = await response.json()
    setDrone(name)
    setDefaults(nextDefaults)
    setError(null)
    onDefaultsLoaded?.(nextDefaults)
    return nextDefaults
  }, [onDefaultsLoaded])

  useEffect(() => {
    const controller = new AbortController()

    async function loadCatalog() {
      try {
        const response = await fetch('/drones/', { signal: controller.signal })
        if (!response.ok) {
          throw new Error('Не удалось загрузить список БПЛА')
        }

        const droneList = await response.json()
        setDrones(droneList)

        if (droneList.length > 0) {
          await loadDroneDefaults(droneList[0].name, controller.signal)
        }
      } catch (error) {
        if (error.name === 'AbortError') {
          return
        }
        setError(toErrorMessage(error, 'Не удалось загрузить каталог БПЛА'))
      }
    }

    loadCatalog()
    return () => controller.abort()
  }, [loadDroneDefaults])

  const selectDrone = useCallback(async (name) => {
    try {
      await loadDroneDefaults(name)
    } catch (error) {
      if (error.name === 'AbortError') {
        return
      }
      setError(toErrorMessage(error, 'Не удалось загрузить параметры БПЛА по умолчанию'))
    }
  }, [loadDroneDefaults])

  return {
    drones,
    drone,
    defaults,
    error,
    selectDrone,
  }
}
