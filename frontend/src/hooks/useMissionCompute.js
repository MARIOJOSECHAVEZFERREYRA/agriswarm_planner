import { useCallback, useEffect, useRef, useState } from 'react'

const POLL_INTERVAL_MS = 1500

function toErrorMessage(error, fallback) {
  if (error instanceof Error && error.message) {
    return error.message
  }
  return fallback
}

export function useMissionCompute(onMissionReady) {
  const [mission, setMission] = useState(null)
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState(null)
  const pollTimeoutRef = useRef(null)
  const requestVersionRef = useRef(0)

  const clearPolling = useCallback(() => {
    if (pollTimeoutRef.current !== null) {
      clearTimeout(pollTimeoutRef.current)
      pollTimeoutRef.current = null
    }
  }, [])

  const resetMissionState = useCallback(() => {
    requestVersionRef.current += 1
    clearPolling()
    setMission(null)
    setLoading(false)
    setError(null)
  }, [clearPolling])

  const pollMission = useCallback(async (missionId, requestVersion) => {
    try {
      const response = await fetch(`/mission/${missionId}`)
      if (!response.ok) {
        throw new Error('Не удалось обновить статус миссии')
      }

      const nextMission = await response.json()
      if (requestVersion !== requestVersionRef.current) {
        return
      }

      setMission(nextMission)

      if (nextMission.status === 'completed' || nextMission.status === 'failed') {
        clearPolling()
        if (nextMission.status === 'completed') {
          onMissionReady(nextMission, nextMission.waypoints)
        }
        return
      }

      pollTimeoutRef.current = window.setTimeout(() => {
        void pollMission(missionId, requestVersion)
      }, POLL_INTERVAL_MS)
    } catch (error) {
      if (requestVersion !== requestVersionRef.current) {
        return
      }
      clearPolling()
      setError(toErrorMessage(error, 'Не удалось обновить статус миссии'))
    }
  }, [clearPolling, onMissionReady])

  const computeMission = useCallback(async (payload) => {
    const requestVersion = requestVersionRef.current + 1
    requestVersionRef.current = requestVersion
    clearPolling()
    setMission(null)
    setLoading(true)
    setError(null)

    try {
      const response = await fetch('/mission/compute', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
      })

      if (!response.ok) {
        const apiError = await response.json()
        throw new Error(apiError.detail ?? response.statusText)
      }

      const createdMission = await response.json()
      if (requestVersion !== requestVersionRef.current) {
        return
      }

      setMission(createdMission)

      if (createdMission.status === 'completed' || createdMission.status === 'failed') {
        if (createdMission.status === 'completed') {
          onMissionReady(createdMission, createdMission.waypoints)
        }
        return
      }

      pollTimeoutRef.current = window.setTimeout(() => {
        void pollMission(createdMission.id, requestVersion)
      }, POLL_INTERVAL_MS)
    } catch (error) {
      if (requestVersion !== requestVersionRef.current) {
        return
      }
      setError(toErrorMessage(error, 'Не удалось рассчитать миссию'))
    } finally {
      if (requestVersion === requestVersionRef.current) {
        setLoading(false)
      }
    }
  }, [clearPolling, onMissionReady, pollMission])

  useEffect(() => () => clearPolling(), [clearPolling])

  return {
    mission,
    loading,
    error,
    computeMission,
    resetMissionState,
  }
}
