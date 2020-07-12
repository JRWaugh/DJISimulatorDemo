package com.dji.djisimulatordemo;

import java.util.ArrayList;

import dji.common.flightcontroller.ObstacleDetectionSector;
import dji.common.flightcontroller.ObstacleDetectionSectorWarning;

public class IndoorFlightController {
    enum State {Lost, ApproachingWall, WallFollow}

    ;
    final static float MIN_SAFE_DISTANCE = 1.2f;
    final static float MAX_DISTANCE = 100.0f;
    private State state = State.Lost;
    boolean isFlying;
    private ArrayList<Wall> walls = new ArrayList<>(2);

    private static final String TAG = IndoorFlightController.class.getName();

    public IndoorFlightController() {

    }

    public void updateWalls(final ObstacleDetectionSector[] detectionSectors) {
        // Call this when sectors updated and when going into ATTI mode
        switch (state) {
            case Lost:
                if (!sectorsValid(detectionSectors))
                    state = State.Lost; // No change
                    if (sectorsEqual(detectionSectors)) {
                        // If searching -> w
                    }
                    if (walls.isEmpty()) {
                        {
                            // walls.add(new Wall());
                            if ()
                        }
                    } else {

                    }
                } else {
                    // Sectors not valid.
                }
                if (walls.isEmpty()) {
                    // SET STATE TO SEARCHING
                    if (detectionSectors[0].getObstacleDistanceInMeters() == MAX_DISTANCE) {
                        // ROTATE RIGHT
                    } else if (detectionSectors[3].getObstacleDistanceInMeters() == MAX_DISTANCE) {
                        // ROTATE LEFT
                    } else
                }

        }
    }

    private static boolean sectorsEqual(final ObstacleDetectionSector[] detectionSectors) {
        for (int i = 1; i < detectionSectors.length; ++i)
            if (detectionSectors[0].getObstacleDistanceInMeters() != detectionSectors[i].getObstacleDistanceInMeters())
                return false;
        return true;
    }

    private static boolean sectorDistancesIdeal(final ObstacleDetectionSector[] detectionSectors) {
        for (final ObstacleDetectionSector detectionSector : detectionSectors)
            if (detectionSector.getObstacleDistanceInMeters() < 1.5f || detectionSector.getObstacleDistanceInMeters() > 2.25f)
                return false;
        return true;
    }

    private static boolean sectorsValid(final ObstacleDetectionSector[] detectionSectors) {
        for (final ObstacleDetectionSector detectionSector : detectionSectors)
            if (detectionSector.getWarningLevel() == ObstacleDetectionSectorWarning.INVALID)
                return false;
        return true;
    }
}

