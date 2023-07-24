// Package cartofacade contains the api to call into CGO
package cartofacade

import (
	"context"
	"errors"
	"sync"
	"time"
)

type SlamMode int64

var emptyRequestParams = map[RequestParamType]interface{}{}

// ErrUnableToAcquireLock is the error returned from AddSensorReading when lock can't be acquired.
var ErrUnableToAcquireLock = errors.New("VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK")

// Initialize calls into the cartofacade C code.
func (cf *CartoFacade) Initialize(ctx context.Context, timeout time.Duration, activeBackgroundWorkers *sync.WaitGroup) (SlamMode, error) {
	return 1, nil
}

// Start calls into the cartofacade C code.
func (cf *CartoFacade) Start(ctx context.Context, timeout time.Duration) error {
	return nil
}

// Stop calls into the cartofacade C code.
func (cf *CartoFacade) Stop(ctx context.Context, timeout time.Duration) error {
	return nil
}

// Terminate calls into the cartofacade C code.
func (cf *CartoFacade) Terminate(ctx context.Context, timeout time.Duration) error {
	return nil
}

// AddSensorReading calls into the cartofacade C code.
func (cf *CartoFacade) AddSensorReading(
	ctx context.Context,
	timeout time.Duration,
	sensorName string,
	currentReading []byte,
	readingTimestamp time.Time,
) error {
	return nil
}

type GetPosition struct{}

// GetPosition calls into the cartofacade C code.
func (cf *CartoFacade) GetPosition(ctx context.Context, timeout time.Duration) (GetPosition, error) {
	return GetPosition{}, nil
}

// GetInternalState calls into the cartofacade C code.
func (cf *CartoFacade) GetInternalState(ctx context.Context, timeout time.Duration) ([]byte, error) {
	return []byte{}, nil
}

// GetPointCloudMap calls into the cartofacade C code.
func (cf *CartoFacade) GetPointCloudMap(ctx context.Context, timeout time.Duration) ([]byte, error) {
	return []byte{}, errors.New("unable to cast response from cartofacade to a byte slice")
}

// RequestType defines the carto C API call that is being made.
type RequestType int64

const (
	// initialize represents the viam_carto_init call in c.
	initialize RequestType = iota
	// start represents the viam_carto_start call in c.
	start
	// stop represents the viam_carto_stop call in c.
	stop
	// terminate represents the viam_carto_terminate in c.
	terminate
	// addSensorReading represents the viam_carto_add_sensor_reading in c.
	addSensorReading
	// position represents the viam_carto_get_position call in c.
	position
	// internalState represents the viam_carto_get_internal_state call in c.
	internalState
	// pointCloudMap represents the viam_carto_get_point_cloud_map in c.
	pointCloudMap
)

// RequestParamType defines the type being provided as input to the work.
type RequestParamType int64

const (
	// sensor represents a sensor name input into c funcs.
	sensor RequestParamType = iota
	// reading represents a lidar reading input into c funcs.
	reading
	// timestamp represents the timestamp input into c funcs.
	timestamp
)

// Response defines the result of one piece of work that can be put on the result channel.
type Response struct {
	result interface{}
	err    error
}

/*
CartoFacade exists to ensure that only one go routine is calling into the CGO api at a time to ensure the
go runtime doesn't spawn multiple OS threads, which would harm performance.
*/
type CartoFacade struct {
}

// RequestInterface defines the functionality of a Request.
// It should not be used outside of this package but needs to be public for testing purposes.
type RequestInterface interface {
	doWork(q *CartoFacade) (interface{}, error)
}

// Interface defines the functionality of a CartoFacade instance.
// It should not be used outside of this package but needs to be public for testing purposes.
type Interface interface {
	request(
		ctxParent context.Context,
		requestType RequestType,
		inputs map[RequestParamType]interface{}, timeout time.Duration,
	) (interface{}, error)
	startCGoroutine(
		ctx context.Context,
		activeBackgroundWorkers *sync.WaitGroup,
	)

	Initialize(
		ctx context.Context,
		timeout time.Duration,
		activeBackgroundWorkers *sync.WaitGroup,
	) (SlamMode, error)
	Start(
		ctx context.Context,
		timeout time.Duration,
	) error
	Stop(
		ctx context.Context,
		timeout time.Duration,
	) error
	Terminate(
		ctx context.Context,
		timeout time.Duration,
	) error
	AddSensorReading(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error
	GetPosition(
		ctx context.Context,
		timeout time.Duration,
	) (GetPosition, error)
	GetInternalState(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error)
	GetPointCloudMap(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error)
}

// Request defines all of the necessary pieces to call into the CGo API.
type Request struct {
	responseChan  chan Response
	requestType   RequestType
	requestParams map[RequestParamType]interface{}
}

// New instantiates the Cartofacade struct which limits calls into C.
func New() CartoFacade {
	return CartoFacade{}
}
