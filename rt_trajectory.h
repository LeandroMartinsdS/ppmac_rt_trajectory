#ifndef RT_TRAJECTORY_H
#define RT_TRAJECTORY_H

#include "../Include/pp_proj.h"

#define BuffLen       1000  //Allocated Buffer Length
#define int2double    (double)sizeof(int)/sizeof(double)
#define NUM_MOTORS    8
#define NUM_BUFFERS   2

#define MASTER_ECT_BASE 18

#define AddPtrVar(i, x)     SetPtrVar(i, GetPtrVar(i)+x)

enum ptrM bufferAddr[NUM_BUFFERS] = {BufferAdr_A, BufferAdr_B};
enum ptrM bufferFill[NUM_BUFFERS] = {BufferFill_A, BufferFill_B};
enum State {
    IDLE,
    SETUP,
    RUN,
    STOP,
    KILL,
    ABORT_ERROR,
};

inline void parseAxes(unsigned int axes);
inline void setAddresses(unsigned int bufferAddr, int **pushm_user, double **pushm_positions);
inline void setOutput(unsigned int userCmd);
inline void toggleBuffer(unsigned int buffer);
inline void setMasterCtrl(unsigned int value);
void rt_trajectory(struct MotorData *Mptr);
//EXPORT_SYMBOL(rt_trajectory);

///////////////////////////////////////////////////////////////////////////////////////////////////

inline void parseAxes(unsigned int axes) {
    if(axes & ~((1U << NUM_MOTORS)-1)) {
        SetPtrVar(Traj_Status,3);
        SetPtrVar(Error,1);
        return;
   }
    SetPtrVar(AxesParser,axes);
}

inline void setAddresses(unsigned int bufferAddr, int **pushm_user, double **pushm_positions) {
//    if(bufferAddr > ...) {
//        SetPtrVar(Traj_Status,3);
//        SetPtrVar(Error,1);
//        return;
//    }

    SetPtrVar(CurrentIndex,0);
    // User Cmd Address - MSB of 1st byte
    *pushm_user = (int *) pushm + bufferAddr/sizeof(int) + 1;
    // Positions Addresses
    *pushm_positions = (double *) pushm + bufferAddr/sizeof(double) + 1;
}

inline void setOutput(unsigned int userCmd) {
//   if(usrCmd & ~((1U << 3)-1)) {
//        SetPtrVar(Traj_Status,3);
//        SetPtrVar(Error,1);
//        return;
//    }


    SetPtrVar(32, (userCmd)      & 1);
    SetPtrVar(33, (userCmd >> 1) & 1);
    SetPtrVar(34, (userCmd >> 2) & 1);
}

inline void toggleBuffer(unsigned int buffer) {
    unsigned int new_buffer;

//    if(buffer & ~((1U << NUM_BUFFERS)-1)) {
//        SetPtrVar(Traj_Status,3);
//        SetPtrVar(Error,1);
//        return;
//    }

    new_buffer = (buffer+1)%NUM_BUFFERS;
    SetPtrVar(CurrentBuffer, new_buffer);
    SetPtrVar(CurrentBufferAdr, GetPtrVar(bufferAddr[new_buffer]));
    SetPtrVar(CurrentBufferFill, GetPtrVar(bufferFill[new_buffer]));
}

inline void setMasterCtrl(unsigned int value) {
    unsigned int motor;

//    if(value & ~((1U << 2)-1)) {
//        SetPtrVar(Traj_Status,3);
//        SetPtrVar(Error,1);
//        return;
//    }

    for(motor=1; motor<=NUM_MOTORS; motor++) {
        // Motor check
        if(GetPtrArrayVar(AxesParser, motor)) {
            pshm->Motor[motor].MasterCtrl = value;
        }
    }
}

inline void setCurrentPos(double *pushm, double *pushm_positions) {
    unsigned int motor;
    double *pushm_currentPos=(double *)pushm + 19;

    for(motor=1; motor<=NUM_MOTORS; motor++) {
        if(GetPtrArrayVar(AxesParser, motor) && pshm->Motor[motor].ClosedLoop) {
            *pushm_currentPos = *pushm_positions;
        }
        pushm_positions += 1;
        pushm_currentPos += 1;
    }
}

inline void resetCurrentPos(double *pushm) {
    unsigned int motor;
    double *pushm_currentPos;

    for(motor=1; motor<=NUM_MOTORS; motor++) {
        if(GetPtrArrayVar(AxesParser, motor)) {
            pushm_currentPos = (double *)pushm + MASTER_ECT_BASE + motor;
            *pushm_currentPos = 0;
        }
    }
}

void rt_trajectory(struct MotorData *Mptr) {
    static enum State state_ = IDLE;
    static int *pushm_user;
    static double *pushm_positions;
    static double *pushm_currentPos;
    unsigned int motor = 0;

    if(Mptr->ClosedLoop)
    {
        switch(state_) {
            case IDLE:
                SetPtrVar(Traj_Status,1);
                SetPtrVar(AbortTrigger,0);
                SetPtrVar(Error,0);

                SetPtrVar(TotalPoints,0);

                SetPtrVar(CurrentBuffer, 0);
                SetPtrVar(CurrentBufferAdr, GetPtrVar(bufferAddr[0]));
                SetPtrVar(CurrentBufferFill, GetPtrVar(bufferFill[0]));

                parseAxes(GetPtrVar(Axes));

                // MasterEnc approach
                // Reset User data
                resetCurrentPos(pshm);
//                for(motor=1; motor<=NUM_MOTORS; motor++) {
//                    if(GetPtrArrayVar(AxesParser, motor)) {
//                        pushm_currentPos = (double *)pushm + MASTER_ECT_BASE + motor;
//                        *pushm_currentPos = 0;
//                    }
//                }

                // Set MasterCtrl=1(normal) or 3(offset)
                setMasterCtrl(1);

                state_ = SETUP;
                break;

            case SETUP:
                setAddresses(GetPtrVar(CurrentBufferAdr), &pushm_user, &pushm_positions);

                state_ = RUN;
                break;

            case RUN:
                // Run State has to be capable of execute everything in a single Servo cycle
                if(GetPtrVar(AbortTrigger) != 0 || GetPtrVar(Error) != 0) {
                    state_ = ABORT_ERROR;
                    break;
                }

                if(GetPtrVar(CurrentBufferFill) > 0) {
                    if(GetPtrVar(CurrentIndex) == GetPtrVar(CurrentBufferFill)) {
                        if(GetPtrVar(CurrentBufferFill) < GetPtrVar(BufferLength)){
                            state_ = STOP;
                            break;
                        }
                        else {
                            // state_ = TOGGLE; - OLD
                            toggleBuffer(GetPtrVar(CurrentBuffer));
                            // state_ = SETUP; - OLD
                            setAddresses(GetPtrVar(CurrentBufferAdr), &pushm_user, &pushm_positions);
                        }
                    }
                    //setCurrentPos(pushm, pushm_positions);
                    for(motor=1; motor<=NUM_MOTORS; motor++) {
                        // Motor check
                        if(GetPtrArrayVar(AxesParser,motor) && pshm->Motor[motor].ClosedLoop) {
                            pushm_currentPos = (double *)pushm + MASTER_ECT_BASE + motor;
                            *pushm_currentPos = *pushm_positions;
                        }
                        // Increment pointer to next motor
                        pushm_positions += 1;
                    }
                    setOutput(*pushm_user);

                    // Skip for next point
                    // Skip 8 bytes, that contain UserCmd
                    pushm_positions+=1;     // double
                    // Skip 8*NUM_MOTORS bytes (containing positions) for next point
                    pushm_user += (NUM_MOTORS+1)*2; // integer

                    AddPtrVar(TotalPoints, 1);
                    AddPtrVar(CurrentIndex, 1);

                }
                else {
                    state_ = STOP;
                }
                break;

            case STOP:
                setMasterCtrl(0);

                state_ = KILL;
                break;

            case ABORT_ERROR:
                // Controlled stop, END
                // MasterMaxSpeed and MasterMaxAccel can be reconfigured here
                // pshm->Motor[1].MasterMaxSpeed=50
                // pshm->Motor[1].MasterMaxAccel=100
                setMasterCtrl(0);
                break;
            case KILL:
                // Kill Mptr motor (usually Motor[0])
                Mptr->ClosedLoop = 0;
                AmpDisable(0);
                break;
        }

    }
    else {
        resetCurrentPos(pshm);
        SetPtrVar(AxesParser, 0);
        state_ = IDLE;
    }
}
EXPORT_SYMBOL(rt_trajectory);

#endif // RT_TRAJECTORY_H