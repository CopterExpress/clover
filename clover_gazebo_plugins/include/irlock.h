
#ifndef IRLOCK_H_
#define IRLOCK_H_

/** irlock constants */

#define IRLOCK_RES_X 320
#define IRLOCK_RES_Y 200

#define IRLOCK_CENTER_X				(IRLOCK_RES_X/2)			// the x-axis center pixel position
#define IRLOCK_CENTER_Y				(IRLOCK_RES_Y/2)			// the y-axis center pixel position

#define IRLOCK_FOV_X (60.0f*M_PI_F/180.0f)
#define IRLOCK_FOV_Y (35.0f*M_PI_F/180.0f)

#define IRLOCK_TAN_HALF_FOV_X 0.57735026919f // tan(0.5 * 60 * pi/180)
#define IRLOCK_TAN_HALF_FOV_Y 0.31529878887f // tan(0.5 * 35 * pi/180)

#define IRLOCK_TAN_ANG_PER_PIXEL_X	(2*IRLOCK_TAN_HALF_FOV_X/IRLOCK_RES_X)
#define IRLOCK_TAN_ANG_PER_PIXEL_Y	(2*IRLOCK_TAN_HALF_FOV_Y/IRLOCK_RES_Y)

/** irlock data structures */

#define IRLOCK_OBJECTS_MAX	5	/** up to 5 objects can be detected/reported **/

struct irlock_target_s {
	uint16_t signature;	/** target signature **/
	float pos_x;	/** x-axis distance from center of image to center of target in units of tan(theta) **/
	float pos_y;	/** y-axis distance from center of image to center of target in units of tan(theta) **/
	float size_x;	/** size of target along x-axis in units of tan(theta) **/
	float size_y;	/** size of target along y-axis in units of tan(theta) **/
};

/** irlock_s structure returned from read calls **/
struct irlock_s {
	uint64_t timestamp; /** microseconds since system start **/
	uint8_t num_targets;
	struct irlock_target_s targets[IRLOCK_OBJECTS_MAX];
};


#endif /* IRLOCK_H_ */