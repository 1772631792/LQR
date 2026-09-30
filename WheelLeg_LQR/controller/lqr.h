#ifndef LQR_H
#define LQR_H

#if defined(_WIN32) && defined(LQR_BUILD_DLL)
#define LQR_API __declspec(dllexport)
#else
#define LQR_API
#endif

typedef struct {
    float pos;         /* m */
    float vel;         /* m/s */
    float pitch;       /* rad, positive forward */
    float pitch_rate;  /* rad/s */
} RobotState;

typedef struct {
    float K[4];
    float output_limit; /* N: total horizontal drive force, NOT motor torque */
} LQRController;

LQR_API void LQR_Init(LQRController *ctrl);
LQR_API float LQR_Update(LQRController *ctrl, const RobotState *state,
                         const RobotState *reference);
LQR_API float LQR_GetSampleTime(void);
LQR_API const char *LQR_GetDesignId(void);
LQR_API unsigned int LQR_GetStateSize(void);
LQR_API unsigned int LQR_GetControllerSize(void);
#endif
