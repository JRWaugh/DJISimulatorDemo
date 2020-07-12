package com.dji.djisimulatordemo;

import dji.common.flightcontroller.ObstacleDetectionSector;

public class DetectionSectorWrapper {
    public DetectionSectorWrapper(final ObstacleDetectionSector[] detectionSectors) {
        mDetectionSectors = detectionSectors;
    }

    public boolean sectorsAllEqual() {
        for (int i = 1; i < mDetectionSectors.length; ++i)
            if (mDetectionSectors[0] != mDetectionSectors[i])
                return false;
        return true;
    }

    public boolean sectorsReliable() {
        float minObstacleDistance = 0.0f;
        float maxObstacleDistance = 0.0f;

        for (final ObstacleDetectionSector detectionSector : mDetectionSectors) {
            minObstacleDistance = minObstacleDistance == 0 ? detectionSector.getObstacleDistanceInMeters() : Math.min(minObstacleDistance, detectionSector.getObstacleDistanceInMeters());
            maxObstacleDistance = maxObstacleDistance == 0 ? detectionSector.getObstacleDistanceInMeters() : Math.max(maxObstacleDistance, detectionSector.getObstacleDistanceInMeters());
        }

        return !(maxObstacleDistance - minObstacleDistance > 50.0f);
    }

    private final ObstacleDetectionSector[] mDetectionSectors;
}
