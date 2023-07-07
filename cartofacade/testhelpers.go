package cartofacade

import (
	"os"
)

// GetTestConfig gets a sample config for testing purposes.
func GetTestConfig(sensor string) (CartoConfig, string, error) {
	dir, err := os.MkdirTemp("", "slam-test")
	if err != nil {
		return CartoConfig{}, "", err
	}

	return CartoConfig{
		Sensors:            []string{sensor, "imu"},
		MapRateSecond:      5,
		DataDir:            dir,
		ComponentReference: "component",
		LidarConfig:        TwoD,
	}, dir, nil
}

// GetBadTestConfig gets a sample config for testing purposes that will cause a failure.
func GetBadTestConfig() CartoConfig {
	return CartoConfig{
		Sensors:     []string{"rplidar", "imu"},
		LidarConfig: TwoD,
	}
}

// GetTestAlgoConfig gets a sample algo config for testing purposes.
func GetTestAlgoConfig() CartoAlgoConfig {
	return CartoAlgoConfig{
		OptimizeOnStart:      false,
		OptimizeEveryNNodes:  3,
		NumRangeData:         100,
		MissingDataRayLength: 25.0,
		MaxRange:             25.0,
		MinRange:             0.2,
		MaxSubmapsToKeep:     3,
		FreshSubmapsCount:    3,
		MinCoveredArea:       1.0,
		MinAddedSubmapsCount: 1,
		OccupiedSpaceWeight:  20.0,
		TranslationWeight:    10.0,
		RotationWeight:       1.0,
	}
}
