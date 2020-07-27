package com.dji.videostreamdecodingsample;

import dji.common.flightcontroller.ObstacleDetectionSector;
import dji.common.flightcontroller.ObstacleDetectionSectorWarning;

public class DetectionEvent {
    public DetectionEvent(final ObstacleDetectionSector[] detectionSectors) {
        mDetectionSectors = detectionSectors;
    }

    private enum SectorState {
        Invalid,
        EquallyObstructed,
        LeftSideObstructed,
        RightSideObstructed,
        Unobstructed
    }

    public boolean sectorsAllEqual() {
        for (int i = 1; i < mDetectionSectors.length; ++i)
            if (mDetectionSectors[0] != mDetectionSectors[i])
                return false;
        return true;
    }

    public boolean sectorsAllValid() {
        for (ObstacleDetectionSector mDetectionSector : mDetectionSectors)
            if (mDetectionSector.getWarningLevel() == ObstacleDetectionSectorWarning.INVALID)
                return false;
        return true;
    }

    public float sectorNearest() {
        float minObstacleDistance = 0.0f;
        for (final ObstacleDetectionSector detectionSector : mDetectionSectors) {
            final float obstacleDistanceInMeters = (detectionSector.getWarningLevel() != ObstacleDetectionSectorWarning.INVALID) ? detectionSector.getObstacleDistanceInMeters() : 0.0f;
            minObstacleDistance = minObstacleDistance == 0.0f ? obstacleDistanceInMeters  : Math.min(minObstacleDistance, obstacleDistanceInMeters);
        }
        return minObstacleDistance;
    }

    private final ObstacleDetectionSector[] mDetectionSectors;
}
