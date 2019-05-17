/*    This file is distributed with 42,                               */
/*    the (mostly harmless) spacecraft dynamics simulation            */
/*    created by Eric Stoneking of NASA Goddard Space Flight Center   */

/*    Copyright 2010 United States Government                         */
/*    as represented by the Administrator                             */
/*    of the National Aeronautics and Space Administration.           */

/*    No copyright is claimed in the United States                    */
/*    under Title 17, U.S. Code.                                      */

/*    All Other Rights Reserved.                                      */


#include "42.h"
#include "SerialCommunication.h"



void AcFsw(struct AcType *AC);

/* #ifdef __cplusplus
** namespace _42 {
** using namespace Kit;
** #endif
*/

/**********************************************************************/
long FswCmdInterpreter(char CmdLine[512],double *CmdTime)
{
      long NewCmdProcessed = FALSE;
      long Isc,Ib,Ig,Iw,It,i,Isct,Ibt,Ithr;
      char response[80];
      char FrameChar;
      long Frame;
      struct CmdType *Cmd;
      struct CmdVecType *CV;
      double q[4],Ang[3],C[3][3],VecR[3],Vec[3],VecH[3];
      double RA,Dec;
      double Lng,Lat,Alt;
      double wc,amax,vmax;
      long RotSeq;
      char VecString[20],TargetString[20];

      if (sscanf(CmdLine,"%lf SC[%ld] qrn = [%lf %lf %lf %lf]",
         CmdTime,&Isc,&q[0],&q[1],&q[2],&q[3]) == 6) {
         NewCmdProcessed = TRUE;
         Cmd = &SC[Isc].AC.Cmd;
         Cmd->Parm = PARM_QUATERNION;
         Cmd->Frame = FRAME_N;
         for(i=0;i<4;i++) Cmd->qrn[i] = q[i];
      }

      else if (sscanf(CmdLine,"%lf SC[%ld] qrl = [%lf %lf %lf %lf]",
         CmdTime,&Isc,&q[0],&q[1],&q[2],&q[3]) == 6) {
         NewCmdProcessed = TRUE;
         Cmd = &SC[Isc].AC.Cmd;
         Cmd->Parm = PARM_QUATERNION;
         Cmd->Frame = FRAME_L;
         for(i=0;i<4;i++) Cmd->qrl[i] = q[i];
      }

      else if (sscanf(CmdLine,"%lf SC[%ld] FswTag = %s",
         CmdTime,&Isc,response) == 3) {
         NewCmdProcessed = TRUE;
         SC[Isc].FswTag = DecodeString(response);
      }

      else if (sscanf(CmdLine,"%lf SC[%ld] Cmd Angles = [%lf %lf %lf] deg, Seq = %ld wrt %c Frame",
         CmdTime,&Isc,&Ang[0],&Ang[1],&Ang[2],&RotSeq,&FrameChar) == 7) {
         NewCmdProcessed = TRUE;
         Cmd = &SC[Isc].AC.Cmd;
         Cmd->Parm = PARM_EULER_ANGLES;
         if (FrameChar == 'L') Cmd->Frame = FRAME_L;
         else Cmd->Frame = FRAME_N;
         for(i=0;i<3;i++) Cmd->Ang[i] = Ang[i]*D2R;
         Cmd->RotSeq = RotSeq;
         A2C(RotSeq,Ang[0]*D2R,Ang[1]*D2R,Ang[2]*D2R,C);
         if (Cmd->Frame == FRAME_L) C2Q(C,Cmd->qrl);
         else C2Q(C,Cmd->qrn);
      }

      else if (sscanf(CmdLine,"%lf SC[%ld].G[%ld] Cmd Angles = [%lf %lf %lf] deg",
         CmdTime,&Isc,&Ig,&Ang[0],&Ang[1],&Ang[2]) == 6) {
         NewCmdProcessed = TRUE;
         Cmd = &SC[Isc].AC.G[Ig].Cmd;
         Cmd->Parm = PARM_EULER_ANGLES;
         for(i=0;i<3;i++) Cmd->Ang[i] = Ang[i]*D2R;
      }

      else if (sscanf(CmdLine,"%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at RA = %lf deg, Dec = %lf deg",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],&RA,&Dec) == 9) {
         NewCmdProcessed = TRUE;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_DIRECTION;
         CV->Frame = FRAME_N;
         UNITV(VecR);
         for(i=0;i<3;i++) CV->R[i] = VecR[i];
         CV->N[0] = cos(RA*D2R)*cos(Dec*D2R);
         CV->N[1] = sin(RA*D2R)*cos(Dec*D2R);
         CV->N[2] = sin(Dec*D2R);
      }

      else if (sscanf(CmdLine,
         "%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at World[%ld] Lng = %lf deg, Lat = %lf deg, Alt = %lf km",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],&Iw,&Lng,&Lat,&Alt) == 11) {
         NewCmdProcessed = TRUE;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         Cmd->Frame = FRAME_N;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_TARGET;
         CV->Frame = FRAME_N;
         CV->TrgType = TARGET_WORLD;
         CV->TrgWorld = Iw;
         UNITV(VecR);
         for(i=0;i<3;i++) CV->R[i] = VecR[i];
         CV->W[0] = (World[Iw].rad+1000.0*Alt)*cos(Lng*D2R)*cos(Lat*D2R);
         CV->W[1] = (World[Iw].rad+1000.0*Alt)*sin(Lng*D2R)*cos(Lat*D2R);
         CV->W[2] = (World[Iw].rad+1000.0*Alt)*sin(Lat*D2R);
      }

      else if (sscanf(CmdLine,"%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at World[%ld]",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],&Iw) == 8) {
         NewCmdProcessed = TRUE;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         Cmd->Frame = FRAME_N;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_TARGET;
         CV->Frame = FRAME_N;
         CV->TrgType = TARGET_WORLD;
         CV->TrgWorld = Iw;
         UNITV(VecR);
         for(i=0;i<3;i++) CV->R[i] = VecR[i];
         for(i=0;i<3;i++) CV->W[i] = 0.0;
      }

      else if (sscanf(CmdLine,"%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at GroundStation[%ld]",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],&It) == 8) {
         NewCmdProcessed = TRUE;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         Cmd->Frame = FRAME_N;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_TARGET;
         CV->Frame = FRAME_N;
         CV->TrgType = TARGET_WORLD;
         CV->TrgWorld = GroundStation[It].World;
         UNITV(VecR);
         for(i=0;i<3;i++) CV->R[i] = VecR[i];
         for(i=0;i<3;i++) CV->W[i] = GroundStation[It].PosW[i];
      }

      else if (sscanf(CmdLine,"%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at SC[%ld].B[%ld] point [%lf %lf %lf]",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],&Isct,&Ibt,&Vec[0],&Vec[1],&Vec[2]) == 12) {
         NewCmdProcessed = TRUE;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         Cmd->Frame = FRAME_N;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_TARGET;
         CV->Frame = FRAME_N;
         CV->TrgType = TARGET_BODY;
         CV->TrgSC = Isct;
         CV->TrgBody = Ibt;
         CopyUnitV(VecR,CV->R);
         for(i=0;i<3;i++) CV->T[i] = Vec[i];
      }

      else if (sscanf(CmdLine,"%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at SC[%ld]",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],&Isct) == 8) {
         NewCmdProcessed = TRUE;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         Cmd->Frame = FRAME_N;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_TARGET;
         CV->Frame = FRAME_N;
         CV->TrgType = TARGET_SC;
         CV->TrgSC = Isct;
         CopyUnitV(VecR,CV->R);
      }

      else if (sscanf(CmdLine,"%lf Point SC[%ld].B[%ld] %s Vector [%lf %lf %lf] at %s",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],TargetString) == 8) {
         NewCmdProcessed = TRUE;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         Cmd->Frame = FRAME_N;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_TARGET;
         CV->Frame = FRAME_N;
         if (!strcmp(TargetString,"EARTH")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = EARTH;
         }
         else if (!strcmp(TargetString,"MOON")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = LUNA;
         }
         else if (!strcmp(TargetString,"LUNA")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = LUNA;
         }
         else if (!strcmp(TargetString,"MERCURY")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = MERCURY;
         }
         else if (!strcmp(TargetString,"VENUS")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = VENUS;
         }
         else if (!strcmp(TargetString,"MARS")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = MARS;
         }
         else if (!strcmp(TargetString,"JUPITER")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = JUPITER;
         }
         else if (!strcmp(TargetString,"SATURN")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = SATURN;
         }
         else if (!strcmp/*  This simple control law is suitable for rapid prototyping.        */(TargetString,"URANUS")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = URANUS;
         }
         else if (!strcmp(TargetString,"NEPTUNE")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = NEPTUNE;
         }
         else if (!strcmp(TargetString,"PLUTO")) {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = PLUTO;
         }
         else if (!strcmp(TargetString,"VELOCITY")) {
            CV->TrgType = TARGET_VELOCITY;
         }
         else if (!strcmp(TargetString,"MAGFIELD")) {
            CV->TrgType = TARGET_MAGFIELD;
         }
         else if (!strcmp(TargetString,"TDRS")) {
            CV->TrgType = TARGET_TDRS;
         }
         else {
            CV->TrgType = TARGET_WORLD;
            CV->TrgWorld = SOL;
         }
         UNITV(VecR);
         for(i=0;i<3;i++) CV->R[i] = VecR[i];
      }

      else if (sscanf(CmdLine,"%lf Align SC[%ld].B[%ld] %s Vector [%lf %lf %lf] with SC[%ld].B[%ld] vector [%lf %lf %lf]",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],&Isct,&Ibt,&Vec[0],&Vec[1],&Vec[2]) == 12) {
         NewCmdProcessed = TRUE;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         Cmd->Frame = FRAME_N;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_DIRECTION;
         CV->Frame = FRAME_B;
         CV->TrgType = TARGET_BODY;
         CV->TrgSC = Isct;
         CV->TrgBody = Ibt;
         CopyUnitV(VecR,CV->R);
         for(i=0;i<3;i++) CV->T[i] = Vec[i];
      }

      else if (sscanf(CmdLine,"%lf Align SC[%ld].B[%ld] %s Vector [%lf %lf %lf] with %c-frame Vector [%lf %lf %lf]",
         CmdTime,&Isc,&Ib,VecString,&VecR[0],&VecR[1],&VecR[2],&FrameChar,&Vec[0],&Vec[1],&Vec[2]) == 11) {
         NewCmdProcessed = TRUE;
         if (FrameChar == 'L') Frame = FRAME_L;
         else if (FrameChar == 'H') {
            Frame = FRAME_N;
            for(i=0;i<3;i++) VecH[i] = Vec[i];
            MxV(World[Orb[SC[Isc].RefOrb].World].CNH,VecH,Vec);
         }
         else Frame = FRAME_N;
         if (Ib == 0) {
            Cmd = &SC[Isc].AC.Cmd;
         }
         else {
            Ig = SC[Isc].B[Ib].Gin;
            Cmd = &SC[Isc].AC.G[Ig].Cmd;
         }
         Cmd->Parm = PARM_VECTORS;
         if (!strcmp(VecString,"Primary")) CV = &Cmd->PriVec;
         else CV = &Cmd->SecVec;
         CV->Mode = CMD_DIRECTION;
         CV->Frame = Frame;
         UNITV(VecR);
         UNITV(Vec);
         for(i=0;i<3;i++) CV->R[i] = VecR[i];
         if (Frame == FRAME_L) {
            for(i=0;i<3;i++) CV->L[i] = Vec[i];
         }
         else {
            for(i=0;i<3;i++) CV->N[i] = Vec[i];
         }
      }

      else if (sscanf(CmdLine,"%lf SC[%ld].Thr[%ld] %s",
         CmdTime,&Isc,&Ithr,response) == 4) {
         NewCmdProcessed = TRUE;
         if (DecodeString(response))
            SC[Isc].AC.Thr[Ithr].PulseWidthCmd = SC[Isc].AC.DT;
         else
            SC[Isc].AC.Thr[Ithr].PulseWidthCmd = 0.0;
      }

      else if (sscanf(CmdLine,"Event Eclipse Entry SC[%ld] qrl = [%lf %lf %lf %lf]",
         &Isc,&q[0],&q[1],&q[2],&q[3]) == 5) {
         *CmdTime = SimTime+DTSIM; /* Allows exiting while loop in CmdInterpreter */
         if (SC[Isc].Eclipse) { /* Will pend on this command until this condition is true */
            NewCmdProcessed = TRUE;
            Cmd = &SC[Isc].AC.Cmd;
            Cmd->Parm = PARM_QUATERNION;
            Cmd->Frame = FRAME_L;
            for(i=0;i<4;i++) Cmd->qrl[i] = q[i];
         }
      }
      else if (sscanf(CmdLine,"Event Eclipse Exit SC[%ld] qrl = [%lf %lf %lf %lf]",
         &Isc,&q[0],&q[1],&q[2],&q[3]) == 5) {
         *CmdTime = SimTime+DTSIM; /* Allows exiting while loop in CmdInterpreter */
         if (!SC[Isc].Eclipse) { /* Will pend on this command until this condition is true */
            NewCmdProcessed = TRUE;
            Cmd = &SC[Isc].AC.Cmd;
            Cmd->Parm = PARM_QUATERNION;
            Cmd->Frame = FRAME_L;
            for(i=0;i<4;i++) Cmd->qrl[i] = q[i];
         }
      }

      else if (sscanf(CmdLine,
         "Event Eclipse Entry SC[%ld] Cmd Angles = [%lf %lf %lf] deg, Seq = %ld wrt %c Frame",
         &Isc,&Ang[0],&Ang[1],&Ang[2],&RotSeq,&FrameChar) == 6) {
         *CmdTime = SimTime+DTSIM; /* Allows exiting while loop in CmdInterpreter */
         if (SC[Isc].Eclipse) { /* Will pend on this command until this condition is true */
            NewCmdProcessed = TRUE;
            Cmd = &SC[Isc].AC.Cmd;
            Cmd->Parm = PARM_EULER_ANGLES;
            if (FrameChar == 'L') Cmd->Frame = FRAME_L;
            else Cmd->Frame = FRAME_N;
            for(i=0;i<3;i++) Cmd->Ang[i] = Ang[i]*D2R;
            Cmd->RotSeq = RotSeq;
            A2C(RotSeq,Ang[0]*D2R,Ang[1]*D2R,Ang[2]*D2R,C);
            if (Cmd->Frame == FRAME_L) C2Q(C,Cmd->qrl);
            else C2Q(C,Cmd->qrn);
         }
      }

      else if (sscanf(CmdLine,
         "Event Eclipse Exit SC[%ld] Cmd Angles = [%lf %lf %lf] deg, Seq = %ld wrt %c Frame",
         &Isc,&Ang[0],&Ang[1],&Ang[2],&RotSeq,&FrameChar) == 6) {
         *CmdTime = SimTime+DTSIM; /* Allows exiting while loop in CmdInterpreter */
         if (!SC[Isc].Eclipse) { /* Will pend on this command until this condition is true */
            NewCmdProcessed = TRUE;
            Cmd = &SC[Isc].AC.Cmd;
            Cmd->Parm = PARM_EULER_ANGLES;
            if (FrameChar == 'L') Cmd->Frame = FRAME_L;
            else Cmd->Frame = FRAME_N;
            for(i=0;i<3;i++) Cmd->Ang[i] = Ang[i]*D2R;
            Cmd->RotSeq = RotSeq;
            A2C(RotSeq,Ang[0]*D2R,Ang[1]*D2R,Ang[2]*D2R,C);
            if (Cmd->Frame == FRAME_L) C2Q(C,Cmd->qrl);
            else C2Q(C,Cmd->qrn);
         }
      }

      else if (sscanf(CmdLine,"%lf Set SC[%ld] RampCoastGlide wc = %lf Hz, amax = %lf, vmax = %lf",
         CmdTime,&Isc,&wc,&amax,&vmax) == 5) {
         NewCmdProcessed = TRUE;
         SC[Isc].AC.PrototypeCtrl.wc = wc*TwoPi;
         SC[Isc].AC.PrototypeCtrl.amax = amax;
         SC[Isc].AC.PrototypeCtrl.vmax = vmax;
      }

      else if (sscanf(CmdLine,"%lf Spin SC[%ld] about Primary Vector at %lf deg/sec",
         CmdTime,&Isc,&wc) == 3) {
         NewCmdProcessed = TRUE;
         Cmd = &SC[Isc].AC.Cmd;
         
         Cmd->Parm = PARM_AXIS_SPIN;
         Cmd->SpinRate = wc*D2R;
      }

      return(NewCmdProcessed);
}
/**********************************************************************/
/* Given a relative position and velocity vector, find the angular    */
/* velocity at which the relative position vector is rotating.        */
void RelMotionToAngRate(double RelPosN[3], double RelVelN[3],
                        double wn[3])
{
      double magp,phat[3],Axis[3],Vpar,Vperp[3],magvp;
      long i;

      magp = CopyUnitV(RelPosN,phat);

      VxV(RelPosN,RelVelN,Axis);
      UNITV(Axis);

      Vpar = VoV(RelVelN,phat);
      for(i=0;i<3;i++) Vperp[i] = RelVelN[i]-Vpar*phat[i];
      magvp = MAGV(Vperp);
      for(i=0;i<3;i++) wn[i] = magvp/magp*Axis[i];
}
/**********************************************************************/
void FindCmdVecN(struct SCType *S, struct CmdVecType *CV)
{
      struct WorldType *W;
      double RelPosB[3],vb[3];
      double RelPosN[3],RelPosH[3],RelVelN[3],RelVelH[3];
      double pcmn[3],pn[3],vn[3],ph[3],vh[3];
      double CosPriMerAng,SinPriMerAng;
      double MaxToS,Rhat[3],ToS;
      long It,i;

      switch (CV->TrgType) {
         case TARGET_WORLD:
            W = &World[CV->TrgWorld];
            CosPriMerAng = cos(W->PriMerAng);
            SinPriMerAng = sin(W->PriMerAng);
            pn[0] =  CV->W[0]*CosPriMerAng - CV->W[1]*SinPriMerAng;
            pn[1] =  CV->W[0]*SinPriMerAng + CV->W[1]*CosPriMerAng;
            pn[2] =  CV->W[2];
            vn[0] = -CV->W[0]*SinPriMerAng - CV->W[1]*CosPriMerAng;
            vn[1] =  CV->W[0]*CosPriMerAng - CV->W[1]*SinPriMerAng;
            vn[2] = 0.0;
            if (CV->TrgWorld == Orb[SC->RefOrb].World) {
               for(i=0;i<3;i++) {
                  RelPosN[i] = pn[i] - S->PosN[i];
                  RelVelN[i] = vn[i] - S->VelN[i];
               }
            }
            else {
               MTxV(W->CNH,pn,ph);
               MTxV(W->CNH,vn,vh);
               for(i=0;i<3;i++) {
                  RelPosH[i] = (W->PosH[i]+ph[i])-S->PosH[i];
                  RelVelH[i] = (W->VelH[i]+vh[i])-S->VelH[i];
               }
               MxV(World[Orb[S->RefOrb].World].CNH,RelPosH,RelPosN);
               MxV(World[Orb[S->RefOrb].World].CNH,RelVelH,RelVelN);
            }
            CopyUnitV(RelPosN,CV->N);
            RelMotionToAngRate(RelPosN,RelVelN,CV->wn);
            break;
         case TARGET_SC:
            if (SC[CV->TrgSC].RefOrb == S->RefOrb) {
               for(i=0;i<3;i++) {
                  RelPosN[i] = SC[CV->TrgSC].PosR[i]-S->PosR[i];
                  RelVelN[i] = SC[CV->TrgSC].VelR[i]-S->VelR[i];
               }
            }
            else if (Orb[SC[CV->TrgSC].RefOrb].World == Orb[S->RefOrb].World) {
               for(i=0;i<3;i++) {
                  RelPosN[i] = SC[CV->TrgSC].PosN[i]-S->PosN[i];
                  RelVelN[i] = SC[CV->TrgSC].VelN[i]-S->VelN[i];
               }
            }
            else {
               for(i=0;i<3;i++) {
                  RelPosH[i] = SC[CV->TrgSC].PosH[i]-S->PosH[i];
                  RelVelH[i] = SC[CV->TrgSC].VelH[i]-S->VelH[i];
               }
               MxV(World[Orb[S->RefOrb].World].CNH,RelPosH,RelPosN);
               MxV(World[Orb[S->RefOrb].World].CNH,RelVelH,RelVelN);
            }
            CopyUnitV(RelPosN,CV->N);
            RelMotionToAngRate(RelPosN,RelVelN,CV->wn);
            break;
         case TARGET_BODY:
            MTxV(SC[CV->TrgSC].B[0].CN,SC[CV->TrgSC].cm,pcmn);
            MTxV(SC[CV->TrgSC].B[CV->TrgBody].CN,CV->T,pn);
            for(i=0;i<3;i++) RelPosB[i] = CV->T[i] - SC[CV->TrgSC].B[CV->TrgBody].cm[i];
            VxV(SC[CV->TrgSC].B[CV->TrgBody].wn,RelPosB,vb);
            MTxV(SC[CV->TrgSC].B[CV->TrgBody].CN,vb,vn);
            for(i=0;i<3;i++) {
               pn[i] += SC[CV->TrgSC].B[CV->TrgBody].pn[i]-pcmn[i];
               vn[i] += SC[CV->TrgSC].B[CV->TrgBody].vn[i];
            }
            if (SC[CV->TrgSC].RefOrb == S->RefOrb) {
               for(i=0;i<3;i++) {
                  RelPosN[i] = SC[CV->TrgSC].PosR[i] + pn[i] - S->PosR[i];
                  RelVelN[i] = SC[CV->TrgSC].VelR[i] + vn[i] - S->VelR[i];
               }
            }
            else if (Orb[SC[CV->TrgSC].RefOrb].World == Orb[S->RefOrb].World) {
               for(i=0;i<3;i++) {
                  RelPosN[i] = SC[CV->TrgSC].PosN[i] + pn[i] - S->PosN[i];
                  RelVelN[i] = SC[CV->TrgSC].VelN[i] + vn[i] - S->VelN[i];
               }
            }
            else {
               MTxV(World[Orb[SC[CV->TrgSC].RefOrb].World].CNH,pn,ph);
               MTxV(World[Orb[SC[CV->TrgSC].RefOrb].World].CNH,vn,vh);
               for(i=0;i<3;i++) {
                  RelPosH[i] = SC[CV->TrgSC].PosH[i] + ph[i] - S->PosH[i];
                  RelVelH[i] = SC[CV->TrgSC].VelH[i] + vh[i] - S->VelH[i];
               }
               MxV(World[Orb[S->RefOrb].World].CNH,RelPosH,RelPosN);
               MxV(World[Orb[S->RefOrb].World].CNH,RelVelH,RelVelN);
            }
            CopyUnitV(RelPosN,CV->N);
            RelMotionToAngRate(RelPosN,RelVelN,CV->wn);
            break;
         case TARGET_VELOCITY:
            for(i=0;i<3;i++) CV->N[i] = S->VelN[i];
            UNITV(CV->N);
            break;
         case TARGET_MAGFIELD:
            for(i=0;i<3;i++) CV->N[i] = S->bvn[i];
            UNITV(CV->N);
            break;
         case TARGET_TDRS:
            CV->N[0] = 0.0;
            CV->N[1] = 0.0;
            CV->N[2] = 1.0;
            for(i=0;i<3;i++) CV->wn[i] = 0.0;
            MaxToS = -2.0; /* Bogus */
            CopyUnitV(S->PosN,Rhat);
            /* Aim at TDRS closest to Zenith */
            for(It=0;It<10;It++) {
               if (Tdrs[It].Exists) {
                  for(i=0;i<3;i++)
                     RelPosN[i] = Tdrs[It].PosN[i] - S->PosN[i];
                  UNITV(RelPosN);
                  ToS = VoV(RelPosN,Rhat);
                  if (ToS > MaxToS) {
                     MaxToS = ToS;
                     for(i=0;i<3;i++) CV->N[i] = RelPosN[i];
                  }
               }
            }
            break;
         default:
            break;
      }
}
/**********************************************************************/
void ThreeAxisAttitudeCommand(struct SCType *S)
{
      struct JointType *G;
      struct BodyType *B;
      struct CmdType *Cmd;
      struct CmdVecType *PV, *SV;
      double CRN[3][3],C[3][3],qln[4],Cdot[3][3];
      double PriVecBi[3],SecVecBi[3],PriVecGi[3],SecVecGi[3];
      double PriVecGo[3],SecVecGo[3],CGoGi[3][3];
      long Ig,Bi,i,j;


      Cmd = &S->AC.Cmd;
      PV = &Cmd->PriVec;
      SV = &Cmd->SecVec;

      switch (Cmd->Parm) {
         case PARM_EULER_ANGLES:
            A2C(Cmd->RotSeq,Cmd->Ang[0],Cmd->Ang[1],Cmd->Ang[2],C);
            if (Cmd->Frame == FRAME_L) C2Q(C,Cmd->qrl);
            else C2Q(C,Cmd->qrn);
         case PARM_QUATERNION:
            C2Q(S->CLN,qln);
            if (Cmd->Frame == FRAME_L) {
               QxQ(Cmd->qrl,qln,Cmd->qrn);
               QxV(Cmd->qrn,S->wln,Cmd->wrn);
            }
            break;
         case PARM_VECTORS:
            if (PV->Mode == CMD_TARGET) FindCmdVecN(S,PV);
            else if (PV->Frame == FRAME_N) {
               for(i=0;i<3;i++) PV->wn[i] = 0.0;
            }
            else if (PV->Frame == FRAME_L) {
               MTxV(S->CLN,PV->L,PV->N);
               for(i=0;i<3;i++) PV->wn[i] = S->wln[i];
            }
            else if (PV->Frame == FRAME_B) {
               MTxV(SC[PV->TrgSC].B[PV->TrgBody].CN,PV->T,PV->N);
               MTxV(SC[PV->TrgSC].B[PV->TrgBody].CN,
                  SC[PV->TrgSC].B[PV->TrgBody].wn,PV->wn);
            }

            if (SV->Mode == CMD_TARGET) FindCmdVecN(S,SV);
            else if (SV->Frame == FRAME_N) {
               for(i=0;i<3;i++) SV->wn[i] = 0.0;
            }
            else if (SV->Frame == FRAME_L) {
               MTxV(S->CLN,SV->L,SV->N);
               for(i=0;i<3;i++) SV->wn[i] = S->wln[i];
            }
            else if (SV->Frame == FRAME_B) {
               MTxV(SC[SV->TrgSC].B[SV->TrgBody].CN,SV->T,SV->N);
               MTxV(SC[SV->TrgSC].B[SV->TrgBody].CN,
                  SC[SV->TrgSC].B[SV->TrgBody].wn,SV->wn);
            }
            if (MAGV(PV->N) == 0.0 || MAGV(PV->R) == 0.0)
               printf("Warning: Primary Vector not defined for SC[%ld]\n",S->ID);
            if (MAGV(SV->N) == 0.0 || MAGV(SV->R) == 0.0)
               printf("Warning: Secondary Vector not defined for SC[%ld]\n",S->ID);
            TRIAD(PV->N,SV->N,PV->R,SV->R,CRN);
            C2Q(CRN,Cmd->qrn);
            for(i=0;i<3;i++) {
               for(j=0;j<3;j++) {
                  Cdot[i][j] = (CRN[i][j]-Cmd->OldCRN[i][j])/S->AC.DT;
               }
            }
            CDOT2W(CRN,Cdot,Cmd->wrn);
            for(i=0;i<3;i++) {
               for(j=0;j<3;j++) {
                  Cmd->OldCRN[i][j] = CRN[i][j];
               }
            }
            break;
         default:
            break;
      }

      for(Ig=0;Ig<S->Ng;Ig++) {
         G = &S->G[Ig];
         Bi = G->Bin;
         B = &S->B[Bi];
         Cmd = &S->AC.G[Ig].Cmd;
         PV = &Cmd->PriVec;
         SV = &Cmd->SecVec;

         if (Cmd->Parm == PARM_VECTORS) {
            if (PV->Mode == CMD_TARGET) FindCmdVecN(S,PV);
            else if (PV->Frame == FRAME_L) MTxV(S->CLN,PV->L,PV->N);
            if (SV->Mode == CMD_TARGET) FindCmdVecN(S,SV);
            else if (SV->Frame == FRAME_L) MTxV(S->CLN,SV->L,SV->N);


            if (G->RotDOF == 3) {
               MxV(B->CN,PV->N,PriVecBi);
               MxV(B->CN,SV->N,SecVecBi);
               MxV(G->CGiBi,PriVecBi,PriVecGi);
               MxV(G->CGiBi,SecVecBi,SecVecGi);
               MTxV(G->CBoGo,PV->R,PriVecGo);
               MTxV(G->CBoGo,SV/*  This simple control law is suitable for rapid prototyping.        */->R,SecVecGo);
               TRIAD(PriVecGi,SecVecGi,PriVecGo,SecVecGo,CGoGi);
               C2A(G->RotSeq,CGoGi,&Cmd->Ang[0],
                  &Cmd->Ang[1],&Cmd->Ang[2]);
            }
            else {
               MxV(B->CN,PV->N,PriVecBi);
               PointGimbalToTarget(G->RotSeq,G->CGiBi,G->CBoGo,PriVecBi,
                  PV->R,Cmd->Ang);
            }
         }
      }
}
/**********************************************************************/
void SpinnerCommand(struct SCType *S)
{
      struct CmdType *Cmd;
      struct CmdVecType *PV;
      double MagH;
      long i;
      
      Cmd = &S->AC.Cmd;
      PV = &Cmd->PriVec;
      
      if (PV->Frame != FRAME_N) {
         printf("SpinnerCommand requires that Primary Vector be fixed in N\n");
         exit(1);
      }
      
      FindCmdVecN(S,PV);
      for(i=0;i<3;i++) {
         Cmd->wrn[i] = PV->R[i]*Cmd->SpinRate;
      }
      MxV(S->I,Cmd->wrn,Cmd->Hvr);
      MagH = MAGV(Cmd->Hvr);
      for(i=0;i<3;i++) {
         Cmd->Hvn[i] = PV->N[i]*MagH;
      }
      
}
/**********************************************************************/
/* This function copies needed parameters from the SC structure to    */
/* the AC structure.                                                 */
void InitAC(struct SCType *S)
{
      long Ig,i,j,k;
      struct AcType *AC;
      double **A,**Aplus;
      double r[3];

      AC = &S->AC;

      AC->Init = 1;

      /* Time, Mass */
      AC->DT = DTSIM;
      AC->mass = S->mass;
      for (i=0;i<3;i++) {
         AC->cm[i] = S->cm[i];
         for(j=0;j<3;j++) {
            AC->MOI[i][j] = S->I[i][j];
         }
      }
      
      /* Joints */
      AC->Ng = S->Ng;
      if (AC->Ng > 0) {
         AC->G = (struct AcJointType *) calloc(AC->Ng,sizeof(struct AcJointType));
         for(Ig=0;Ig<AC->Ng;Ig++) {
            AC->G[Ig].IsUnderActiveControl = TRUE;
            AC->G[Ig].IsSpherical = S->G[Ig].IsSpherical;
            AC->G[Ig].RotDOF = S->G[Ig].RotDOF;
            AC->G[Ig].TrnDOF = S->G[Ig].TrnDOF;
            for(i=0;i<3;i++) {
               for(j=0;j<3;j++) {
                  AC->G[Ig].CGiBi[i][j] = S->G[Ig].CGiBi[i][j];
                  AC->G[Ig].CBoGo[i][j] = S->G[Ig].CBoGo[i][j];
               }
            }
            AC->G[Ig].RotSeq = S->G[Ig].RotSeq;
            AC->G[Ig].TrnSeq = S->G[Ig].TrnSeq;
         }
      }
      
      /* Gyro Axes */
      AC->Ngyro = S->Ngyro;
      if (AC->Ngyro > 0) {
         AC->Gyro = (struct AcGyroType *) calloc(AC->Ngyro,sizeof(struct AcGyroType));
         for(i=0;i<S->Ngyro;i++) {
            for(j=0;j<3;j++) {
               AC->Gyro[i].Axis[j] = S->Gyro[i].Axis[j];
            }
         }
      }

      /* Magnetometer Axes */
      AC->Nmag = S->Nmag;
      if (AC->Nmag > 0) {
         AC->MAG = (struct AcMagnetometerType *) calloc(AC->Nmag,sizeof(struct AcMagnetometerType));
         for(i=0;i<S->Nmag;i++) {
            for(j=0;j<3;j++) {
               AC->MAG[i].Axis[j] = S->MAG[i].Axis[j];
            }
         }
      }

      /* Coarse Sun Sensors */
      AC->Ncss = S->Ncss;
      if (AC->Ncss > 0) {
         AC->CSS = (struct AcCssType *) calloc(AC->Ncss,sizeof(struct AcCssType));
         for(i=0;i<S->Ncss;i++) {
            for(j=0;j<3;j++) AC->CSS[i].Axis[j] = S->CSS[i].Axis[j];
            AC->CSS[i].Scale = S->CSS[i].Scale;
         }
      }
      
      /* Fine Sun Sensors */
      AC->Nfss = S->Nfss;
      if (AC->Nfss > 0) {
         AC->FSS = (struct AcFssType *) calloc(AC->Nfss,sizeof(struct AcFssType));
         for(k=0;k<S->Nfss;k++) {
            for(i=0;i<3;i++) {
               for(j=0;j<3;j++) AC->FSS[k].CB[i][j] = S->FSS[k].CB[i][j];
            }
            for(i=0;i<4;i++) AC->FSS[k].qb[i] = S->FSS[k].qb[i];
         }
      }

      /* Star Trackers */
      AC->Nst = S->Nst;
      if (AC->Nst > 0) {
         AC->ST = (struct AcStarTrackerType *) calloc(AC->Nst,sizeof(struct AcStarTrackerType));
         for(k=0;k<S->Nst;k++) {
            for(i=0;i<3;i++) {
               for(j=0;j<3;j++) AC->ST[k].CB[i][j] = S->ST[k].CB[i][j];
            }
            for(i=0;i<4;i++) AC->ST[k].qb[i] = S->ST[k].qb[i];
         }
      }

      /* GPS */
      AC->Ngps = S->Ngps;
      if (AC->Ngps > 0) {
         AC->GPS = (struct AcGpsType *) calloc(AC->Ngps,sizeof(struct AcGpsType)); 
      }     
      
      /* Accelerometer Axes */

      /* Wheels */
      AC->Nwhl = S->Nw;
      if (AC->Nwhl > 0) {
         AC->Whl = (struct AcWhlType *) calloc(AC->Nwhl,sizeof(struct AcWhlType));
         A = CreateMatrix(3,AC->Nwhl);
         Aplus = CreateMatrix(AC->Nwhl,3);
         for (i=0;i<S->Nw;i++) {
            for (j=0;j<3;j++) {
               AC->Whl[i].Axis[j] = S->Whl[i].A[j];
               A[j][i] = S->Whl[i].A[j];
            }
         }
         if (S->Nw == 1) {
            for(i=0;i<3;i++) AC->Whl[0].DistVec[i] = AC->Whl[0].Axis[i]; 
         }
         else if (S->Nw >= 2) {
            PINVG(A,Aplus,3,S->Nw);
            for(i=0;i<AC->Nwhl;i++) {
               for(j=0;j<3;j++) {
                  AC->Whl[i].DistVec[j] = Aplus[i][j];
               }
            }
         }
         DestroyMatrix(A,3);
         DestroyMatrix(Aplus,AC->Nwhl);
         for(i=0;i<S->Nw;i++) {
            AC->Whl[i].J = S->Whl[i].J;
            AC->Whl[i].Tmax = S->Whl[i].Tmax;
            AC->Whl[i].Hmax = S->Whl[i].Hmax;
         }
      }

      /* Magnetic Torquer Bars */
      AC->Nmtb = S->Nmtb;
      if (AC->Nmtb > 0) {
         AC->MTB = (struct AcMtbType *) calloc(AC->Nmtb,sizeof(struct AcMtbType));
         A = CreateMatrix(3,AC->Nmtb);
         Aplus = CreateMatrix(AC->Nmtb,3);
         for (i=0;i<S->Nmtb;i++) {
            for (j=0;j<3;j++) {
               AC->MTB[i].Axis[j] = S->MTB[i].A[j];
               A[j][i] = S->MTB[i].A[j];
            }
         }
         if (S->Nmtb == 1) {
            for(i=0;i<3;i++) AC->MTB[0].DistVec[i] = AC->MTB[0].Axis[i]; 
         }
         else if (S->Nmtb >= 2) {
            PINVG(A,Aplus,3,S->Nmtb);
            for(i=0;i<AC->Nmtb;i++) {
               for(j=0;j<3;j++) {
                  AC->MTB[i].DistVec[j] = Aplus[i][j];
               }
            }
         }
         DestroyMatrix(A,3);
         DestroyMatrix(Aplus,AC->Nmtb);
         for(i=0;i<S->Nmtb;i++) {
            AC->MTB[i].Mmax = S->MTB[i].Mmax;
         }
      }

      /* Thrusters */
      AC->Nthr = S->Nthr;
      if (AC->Nthr > 0) {
         AC->Thr = (struct AcThrType *) calloc(AC->Nthr,sizeof(struct AcThrType));
         for(i=0;i<S->Nthr;i++) {
            AC->Thr[i].Fmax = S->Thr[i].Fmax;
            for(j=0;j<3;j++) {
               AC->Thr[i].Axis[j] = S->Thr[i].A[j];
               AC->Thr[i].PosB[j] = S->Thr[i].PosB[j];
               r[j] = AC->Thr[i].PosB[j] - AC->cm[j];
            }
            VxV(r,AC->Thr[i].Axis,AC->Thr[i].rxA);
         }
      }
      
      /* Controllers */
      AC->PrototypeCtrl.Init = 1;
      AC->AdHocCtrl.Init = 1;
      AC->SpinnerCtrl.Init = 1;
      AC->MomBiasCtrl.Init = 1;
      AC->ThreeAxisCtrl.Init = 1;
      AC->IssCtrl.Init = 1;
      AC->CmgCtrl.Init = 1;
      AC->ThrCtrl.Init = 1;
      AC->CfsCtrl.Init = 1;
      
      AC->PrototypeCtrl.wc = 0.05*TwoPi;
      AC->PrototypeCtrl.amax = 0.01;
      AC->PrototypeCtrl.vmax = 0.5*D2R;
}
/**********************************************************************/
/* The effective inertia for a gimbal is assumed to be the moment of  */
/* inertia of the appendage depending from the joint (that is, all    */
/* bodies for which that joint is in the JointPathTable) about that   */
/* joint, with all joints undeflected.                                */
void FindAppendageInertia(long Ig, struct SCType *S,double Iapp[3])
{
      struct DynType *D;
      struct JointType *G;
      double rho[3],CBoBi[3][3],Coi[3][3],Cr[3],rhog[3],Csofar[3][3];
      double CBoG[3][3],IBoG[3][3];
      long Ib,Jg,j,k;

      D = &S->Dyn;

      for(k=0;k<3;k++) Iapp[k] = 0.0;
      for (Ib=1;Ib<S->Nb;Ib++) {
         if (D->JointPathTable[Ib][Ig].InPath) {
            /* Build undeflected rho */
            Jg = S->B[Ib].Gin;
            for(k=0;k<3;k++) rho[k] = 0.0;
            for(j=0;j<3;j++) {
               for(k=0;k<3;k++) CBoBi[j][k] = 0.0;
               CBoBi[j][j] = 1.0;
            }
            while (Jg > Ig) {
               G = &S->G[Jg];
               MxM(G->CBoGo,G->CGiBi,Coi);
               for(k=0;k<3;k++) rho[k] -= G->rout[k];
               MTxV(Coi,rho,Cr);
               for(k=0;k<3;k++) rho[k] = Cr[k] + G->rin[k];
               for(j=0;j<3;j++) {
                  for(k=0;k<3;k++) Csofar[j][k] = CBoBi[j][k];
               }
               MxM(Csofar,Coi,CBoBi);
               Jg = S->B[G->Bin].Gin;
            }
            G = &S->G[Ig];
            for(k=0;k<3;k++) rho[k] -= G->rout[k];
            MTxV(G->CBoGo,rho,rhog);
            MTxM(CBoBi,G->CBoGo,CBoG);
            /* Parallel axis theorem */
            PARAXIS(S->B[Ib].I,CBoG,S->B[Ib].mass,rhog,IBoG);
            /* Accumulate */
            for(k=0;k<3;k++) Iapp[k] += IBoG[k][k];
         }
      }
}
/**********************************************************************/
/*  SlugSat Flight Software      */
void SlugSatFSW(struct SCType *S)
{
	// Check if the STM32 board is plugged in
	if(serial_port == NULL) {
		return;
	}


	//Variables from 42
	struct AcType *AC; //Attitude control type


	//Actuator variables
	static double w_rw[3] = {524, 524, 524}; // Reaction wheel speed
	double w_rw_dot[3];
	double k = 0.7597; // Torque rod constant (based on physical parameters)
	double rwVmax = 8.0, trVmax = 3.3; // Voltage rails
	double maxDip = 1.5;


	// Get AC pointer
	AC = &S->AC;


	// Input / Output to serial
	int sensorFloats = 18; //number of floats to send
	int actuatorFloats = 6; //number of floats to receive
	float serSend[sensorFloats], serRec[actuatorFloats]; //serial send and receive
	float bSer[3], sunSer[3], gyroSer [3]; //Sensor values to send
	double rwPWM[3], trPWM[3]; //Actuator pwm and torques


	// Convert sensor data to floats
	for (int i = 0;i < 3;i++) {
		bSer[i] = (1e6)*( SC[0].bvb[i]); //Magnetic field in micro Tesla (body frame)
		bSer[i] = (float)bSer[i]; //Convert to float
		gyroSer[i] = (float)SC[0].B[0].wn[i]; //Gyro (radians per second)
		if(SC[0].AC.SunValid) {
			sunSer[i] = (float)SC[0].AC.svb[i]; //Solar vector (body frame)
		}
		else {
			sunSer[i] = 0; // Simulate darkness
		}
	}

	// Find position in J2000
	double C_TEME_TETE[3][3], C_TETE_J2000[3][3], posJ2000[3];
	HiFiEarthPrecNute(JulDay, C_TEME_TETE, C_TETE_J2000);
	MxV(C_TETE_J2000, Orb[0].PosN, posJ2000);


	// Compile data into single float
	for (int i = 0;i < 3;i++) {
		serSend[i] = bSer[i];
		serSend[i+3] = gyroSer[i];
		serSend[i+6] = sunSer[i];
		serSend[i+9] = (float)posJ2000[i];
		serSend[i+12] = (float)w_rw[i];
	}

	// Find Julian date
	double JD = AbsTimeToJD(AbsTime);
	double JD_int;
	double JD_frac = modf(JD, &JD_int); // Send JD as a sum of two floats to preserve precision
	serSend[15] = (float)JD_int;
	serSend[16] = (float)JD_frac;
	serSend[17] = (float)SimTime;


	// Send and receive data from flat-sat
	serialHandshake(serial_port);
	serialSendFloats(serial_port, serSend, sensorFloats);
	if(serialReceiveFloats(serial_port, serRec, actuatorFloats) != actuatorFloats) {
		return;
	}

	// Receive string from STM32 (for debugging purposes)
	char string[500];
	int read_string_err = serialReceiveString(serial_port, (uint8_t*)string);

	//Print TX data to verify transmission	
	printf("\nTX:\n");
	
	// Print mag field
	printf("\nMag Field (micro Tesla):\t");
	for(int i = 0;i < 3;i++) {
		printf("%6.2f\t\t", serSend[i]);
	}
	
	// Print gyro
	printf("\nGyro (rad/sec):\t\t\t");
	for(int i = 0;i < 3;i++) {
		printf("%4.4e\t", serSend[i+3]);
	}
	
	// Print solar vector
	printf("\nSolar Vector (normalized): \t");
	for(int i = 0;i < 3;i++) {
		printf("%4.4e\t", serSend[i+6]);
	}

	// Print Pos J2000
	printf("\nPos J2000 (km):\t\t\t");
	for(int i = 0;i < 3;i++) {
		printf("%4.4e\t", serSend[i+9]);
	}
	
	// Print w_rw
	printf("\nw_rw (rad/sec):\t\t\t");
	for(int i = 0;i < 3;i++) {
		printf("%6.2f\t\t", serSend[i+12]);
	}
	
	// Print time
	printf("\nTime:\t\t\t\t");
	for(int i = 0;i < 2;i++) {
		printf("%4.4e\t", serSend[i+15]);
	}
	
	//Print RX data
	printf("\n\nRX:\n");
	
	//Reaction Wheel PWM
	printf("\nReaction Wheel PWM\t");
	for(int i = 0;i < 3;i++) {
		printf("%4.2f\t", serRec[i]);
	}
	
	//Torque Rod PWM
	printf("\nTorque Rod PWM\t\t");
	for(int i = 0;i < 3;i++) {
		printf("%4.2f\t", serRec[i+3]);
	}	
			

	if(read_string_err == 0 && strlen(string) > 0) {
		printf("\n\nPRINT FROM STM32\n%s\nEND PRINT FROM STM32\n", string);
	}

	// Convert to double and split into reaction wheels and torque rods
	for(int i = 0;i < 3;i++) {
		rwPWM[i] = (double)serRec[i];	// Reaction Wheel
		if(rwPWM[i] > 100.0) rwPWM[i] = 100.0;
		else if(rwPWM[i] < -100.0) rwPWM[i] = -100.0;
		trPWM[i] = (double)serRec[i+3]; // Magnetic torque bar
		if(trPWM[i] > 100.0) trPWM[i] = 100.0;
		else if(trPWM[i] < -100.0) trPWM[i] = -100.0;
	}
	//test
	//Reaction Wheel PWM
	printf("\nReaction Wheel PWM\t");
	for(int i = 0;i < 3;i++) {
	printf("%4.2f\t", serRec[i]);
	}

	//Torque Rod PWM
	printf("\nTorque Rod PWM\t\t");
	for(int i = 0;i < 3;i++) {
	printf("%6.4f\t", trPWM[i]);
	}

	// Convert reaction wheel PWM to torque & send to AC
	static double Kt = 0.00713, Ke = 0.00713332454, R = 92.7; // Reaction wheel motor constants
	double sample_dt = 0.1; // Oversampling dt
	double vRw[3], rwTrq[3]; // Reaction wheel voltage and torque


	// Save old RW speed
	double w_rw_old[3] = {w_rw[0], w_rw[1], w_rw[2]};


	// Oversample reaction wheel dynamics
	for(double t = 0;t < AC->DT;t += sample_dt) {
		for(int i = 0;i < 3;i++) {
			vRw[i] = rwVmax*(rwPWM[i]/100.0); // Get voltage across motor
			// double fricTrq = 0.019e-3 +
			rwTrq[i] = (Kt/R)*(vRw[i] - w_rw[i]*Ke); // Find torque from DC motor equation
			w_rw_dot[i] = rwTrq[i]/AC->Whl[i].J; // Find acceleration from torque
			w_rw[i] += w_rw_dot[i]*sample_dt; // Integrate to find reaction wheel speed
		}
	}

	// Print Craft speed
	printf("\nCraft speed:\t");
	for(int i = 0;i < 3;i++) {
		printf("%4.4e\t", S->B[0].wn[i]);
	}

	// Send torque to achieve the correct changed in rotational inertia in the next sim step
	//printf("\nRW torque:\t");
	for(int i = 0;i < 3;i++) {
		AC->Whl[i].Tcmd = AC->Whl[i].J*(w_rw[i] - w_rw_old[i])/AC->DT;
		//printf("%4.4e\t", S->Whl[i].Trq);
	}
	printf("\n\n");


	//Convert torque rod (MTB) PWM to torque & send to AC
	for(int i = 0;i < 3;i++){
		AC->MTB[i].Mcmd = maxDip*trPWM[i]/100.0; //k*trVmax*trPWM[i]/100.0;
		//printf("\nDipole moment: \t%f\t", AC->MTB[i].Mcmd);
		//printf("\n mcmd: %f\n" , AC->MTB[i].Mcmd);
	}
	printf("\n\n");
	
	
	//Calculate Power
	char s1[300], s2[10], state[10];
	char detumble[] = "Detumble", reorient[] = "Reorient", stabilize[] = "Stabilize";
	
	static int init_run = 0;
	static double detumbleEnergy, reorientEnergy, stabilizationEnergy, totalEnergy;
	double rwPower = 0;
	double trPower;
	double totalPower = 0;
	if(init_run == 0){
		detumbleEnergy = 0;
		reorientEnergy = 0;
		stabilizationEnergy = 0;
		totalEnergy = 0;
		init_run =1;
	}

	//Find ACS state
	sscanf(string, "%s -- %s\n", s1, state);
	
	//Actuator parameters, rwVmax = 8.0, trVmax = 3.3; Voltage rails
	double trRes = 15.0; //Estimated Torque rod resistance (Ohms)
	double rwRes = 92.7; //Faulhaber 1509 terminal resistance


	//Reaction Wheel Power
	for(int i = 0;i < 3;i++) {
		double v = fabs(rwVmax*rwPWM[i]/100.0 - w_rw[i]*Ke); // Voltage across the motor
		rwPower += v*v / rwRes;
	}
	printf("\nReaction Wheel Power:\t%6.3f [mW]", 1000*rwPower);

	//Torque Rod Power
	for(int i = 0;i < 3;i++) {

		//Test code
		//printf("\nvMax: \t %f\n", trVmax);


		double v = trVmax* fabs(trPWM[i]) / 100.0;


		//Test code

		//printf("\n Voltage max:\t %f\t and V: \t%f \n", trVmax, v);
		//printf("\ntrPWM: \t%f\t", trPWM[i]);


		trPower += (v*v / trRes);


		//Test code
		//printf("\ntrPower: \t%f\t",trPower[i]);
	}

	printf("\n\nTorque Rod Power:\t %f", 1000*trPower);

	//Test code

	//double trPowTot =  trPower[0] +trPower[1]+trPower[2];
	//printf("\n");

	//printf ("\n Torque rod power (trPower): \t%f\n", trPower );



	//Total Power
	totalPower = rwPower + trPower ;
	printf("\nTotal Power:\t\t%6.3f [mW]\n", 1000*totalPower);

	//Print instantaneous power to file

	//Create and initialize files
	static FILE *instPower, *instP, *stateEnergy, *tEnergy;
	static long First = 1;

	      if (First) {
	         First = 0;
		         instPower = FileOpen(InOutPath,"instPower.42","wt");
		         instP = FileOpen(InOutPath,"instP.42","wt");
		         stateEnergy = FileOpen(InOutPath,"stateEnergy.42","wt");
		         tEnergy = FileOpen(InOutPath,"totalEnergy.42","wt");
	      }
	//Print to file
	fprintf(instPower, "%lf\t %lf\t %lf\n",rwPower, trPower, totalPower);
	fprintf(instP, "%lf\n", totalPower);


	//Detumbling power
	if (strcmp(detumble, state) == 0){
		detumbleEnergy += totalPower*AC->DT;
	}

	//Orientation Power
	if (strcmp(reorient, state) == 0){
		reorientEnergy += totalPower*AC->DT;
	}
	
	//Stabilization Power
	if (strcmp(stabilize, state) == 0){
		stabilizationEnergy += totalPower*AC->DT;
	}


	totalEnergy = detumbleEnergy + reorientEnergy + stabilizationEnergy;

	//Print out power consumption for each state
	printf("\nDetumbling Energy: %3.3f [J]\t \nReorientation Energy: %3.3f [J]\t \nStabilization Energy: %3.3f [J]\n",
			detumbleEnergy, reorientEnergy, stabilizationEnergy);

	fprintf(stateEnergy, "%lf\t %lf\t%lf\n", detumbleEnergy, reorientEnergy, stabilizationEnergy);
	fprintf(tEnergy, "%lf\n", totalEnergy);


}


/**********************************************************************/
#ifdef _AC_STANDALONE_
//#include "../Database/42Messages.c"
#endif
/**********************************************************************/
/*  This function is called at the simulation rate.  Sub-sampling of  */
/*  control loops should be done on a case-by-case basis.             */
/*  Mode handling, command generation, error determination, feedback  */
/*  and failure detection and correction all fall within the scope of */
/*  this file.                                                        */
/**********************************************************************/
void FlightSoftWare(struct SCType *S)
{
      static long First = 1;
      static SOCKET AcSocket;
      int AcPort = 101010;

      switch(S->FswTag){

      case SlugSat_FSW:
              SlugSatFSW(S);
               break;
			   
      }
}



