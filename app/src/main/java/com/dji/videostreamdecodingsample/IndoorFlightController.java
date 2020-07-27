package com.dji.videostreamdecodingsample;

import static java.lang.Math.round;

public class IndoorFlightController {
    /*
    private static SectorState obstructionState(final ObstacleDetectionSector[] detectionSectors) {
        int equalSectors = 0;
        boolean obstructed = false;
        int obstructionWeight = 0;

        for (int i = 0; i < detectionSectors.length; ++i) {
            if (detectionSectors[i].getWarningLevel() == ObstacleDetectionSectorWarning.INVALID)
                return SectorState.Invalid;

            if (detectionSectors[i].getObstacleDistanceInMeters() == DetectionEvent.IDEAL_DISTANCE) {
                obstructed = true;

                if (i <= 1)
                    --obstructionWeight;
                else
                    ++obstructionWeight;

                if (detectionSectors[0].getObstacleDistanceInMeters() == detectionSectors[i].getObstacleDistanceInMeters())
                    ++equalSectors;
            }
        }

        if (equalSectors == 4)
            return SectorState.EquallyObstructed;
        else if (!obstructed)
            return SectorState.Unobstructed;
        else if (obstructionWeight <= 0) {
            return SectorState.LeftSideObstructed;
        } else
            return SectorState.RightSideObstructed;
    }

     */
}

