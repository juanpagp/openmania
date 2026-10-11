using System;
using System.Collections.Generic;

namespace Tmnf.Sim
{
    /// <summary>The 0x48-byte input packet consumed by the input mapper.</summary>
    [Serializable]
    public struct TMNFRaceInputs
    {
        public uint steerLeftTime;     /* 0x00 */
        public uint reserved04;
        public int steerLeft;          /* 0x08 */
        public uint steerRightTime;    /* 0x0c */
        public uint reserved10;
        public int steerRight;         /* 0x14 */
        public uint steerAnalogTime;   /* 0x18 */
        public uint reserved1c;
        public float steerAnalog;      /* 0x20 */
        public uint accelerateTime;    /* 0x24 */
        public uint reserved28;
        public int accelerate;         /* 0x2c */
        public uint brakeTime;         /* 0x30 */
        public uint reserved34;
        public int brake;              /* 0x38 */
        public uint gasAnalogTime;     /* 0x3c */
        /// <summary>Nonzero on the tick a respawn press applies.</summary>
        public uint respawn;           /* 0x40 */
        public float gasAnalog;        /* 0x44 */

        public static TMNFRaceInputs FromBytes(byte[] b)
        {
            TMNFRaceInputs p = default(TMNFRaceInputs);
            p.steerLeftTime = BitsUtil.GetU32(b, 0x00);
            p.reserved04 = BitsUtil.GetU32(b, 0x04);
            p.steerLeft = BitsUtil.GetI32(b, 0x08);
            p.steerRightTime = BitsUtil.GetU32(b, 0x0c);
            p.reserved10 = BitsUtil.GetU32(b, 0x10);
            p.steerRight = BitsUtil.GetI32(b, 0x14);
            p.steerAnalogTime = BitsUtil.GetU32(b, 0x18);
            p.reserved1c = BitsUtil.GetU32(b, 0x1c);
            p.steerAnalog = BitsUtil.GetF32(b, 0x20);
            p.accelerateTime = BitsUtil.GetU32(b, 0x24);
            p.reserved28 = BitsUtil.GetU32(b, 0x28);
            p.accelerate = BitsUtil.GetI32(b, 0x2c);
            p.brakeTime = BitsUtil.GetU32(b, 0x30);
            p.reserved34 = BitsUtil.GetU32(b, 0x34);
            p.brake = BitsUtil.GetI32(b, 0x38);
            p.gasAnalogTime = BitsUtil.GetU32(b, 0x3c);
            p.respawn = BitsUtil.GetU32(b, 0x40);
            p.gasAnalog = BitsUtil.GetF32(b, 0x44);
            return p;
        }

        public byte[] ToBytes()
        {
            byte[] b = new byte[0x48];
            BitsUtil.PutU32(b, 0x00, steerLeftTime);
            BitsUtil.PutU32(b, 0x04, reserved04);
            BitsUtil.PutI32(b, 0x08, steerLeft);
            BitsUtil.PutU32(b, 0x0c, steerRightTime);
            BitsUtil.PutU32(b, 0x10, reserved10);
            BitsUtil.PutI32(b, 0x14, steerRight);
            BitsUtil.PutU32(b, 0x18, steerAnalogTime);
            BitsUtil.PutU32(b, 0x1c, reserved1c);
            BitsUtil.PutF32(b, 0x20, steerAnalog);
            BitsUtil.PutU32(b, 0x24, accelerateTime);
            BitsUtil.PutU32(b, 0x28, reserved28);
            BitsUtil.PutI32(b, 0x2c, accelerate);
            BitsUtil.PutU32(b, 0x30, brakeTime);
            BitsUtil.PutU32(b, 0x34, reserved34);
            BitsUtil.PutI32(b, 0x38, brake);
            BitsUtil.PutU32(b, 0x3c, gasAnalogTime);
            BitsUtil.PutU32(b, 0x40, respawn);
            BitsUtil.PutF32(b, 0x44, gasAnalog);
            return b;
        }
    }

    /// <summary>Immutable curve keys; bounds are the compiled PC=24 positions.</summary>
    [Serializable]
    public sealed class CFuncKeys
    {
        public uint count;
        public float[] positions = new float[0];
        public float[] lowerBounds = new float[0];
        public float[] upperBounds = new float[0];
    }

    /// <summary>A real-valued curve.</summary>
    [Serializable]
    public sealed class CFuncKeysReal
    {
        public CFuncKeys keys = new CFuncKeys();
        public float[] values = new float[0];
        public int interpolation;
    }

    /// <summary>The curve set referenced by the tuning views (21 curves).</summary>
    [Serializable]
    public sealed class CSceneVehicleCarTuningCurveSet
    {
        public CFuncKeysReal accelFromSpeed;
        public CFuncKeysReal rolloverLateralFromSpeed;
        public CFuncKeysReal maxSideFrictionFromSpeed;
        public CFuncKeysReal lateralContactSlowdownFromSpeed;
        public CFuncKeysReal steerSlowdownFromSpeed;
        public CFuncKeysReal rolloverLateralCoefFromAngle;
        public CFuncKeysReal steerDriveTorqueFromSpeed;
        public CFuncKeysReal m4SteerRadiusFromSpeed;
        public CFuncKeysReal m4MaxFrictionForceFromSpeed;
        public CFuncKeysReal m5SlippingAccelFromSpeed;
        public float m5SlippingAccelScale;
        public CFuncKeysReal waterFrictionFromSpeed;
        public float damperMax;
        public float damperMin;
        public CFuncKeysReal m6DamperModulation;
        public CFuncKeysReal m6RearGearAccelFromSpeed;
        public CFuncKeysReal m6RolloverLateralFromSpeedRatio;
        public CFuncKeysReal m6BurnoutRadiusFromSpeed;
        public CFuncKeysReal m6LateralSpeedFromBurnoutRadius;
        public CFuncKeysReal m6DonutRolloverFromSpeed;
        public CFuncKeysReal m6BurnoutRolloverFromSpeed;

        public CSceneVehicleCarTuningCurveSet Clone()
        {
            return (CSceneVehicleCarTuningCurveSet)MemberwiseClone();
        }
    }

    /// <summary>The primary tuning block scalars.</summary>
    [Serializable]
    public sealed class CSceneVehicleCarTuning
    {
        public float damperMax;
        public float damperMin;
        public List<float> gearRatios = new List<float>();
        public List<float> gearUpshift = new List<float>();
        public List<float> gearDownshift = new List<float>();
        public List<float> gearAux = new List<float>();
        public uint gearRatioCount;
        public int engineModel;
        public float forwardSpeedLimitScale;   // game +0x02c
        public float engineRpmAccel;
        public float engineRpmDecel;
        public float engineRpmReverseAccel;
        public float engineRpmHighDecel;
        public float engineRpmLowAccel;
        public float engineRpmFollowAccel;
        public float engineRpmTurboDecel;
        public float speed32c;
        public float speed330;
        public float speed334;
        public float speed338;
        public int suspensionModel;
        public float suspensionStiffness;
        public float suspensionDamping;
        public float suspensionRestLength;
        public float suspensionScale;
    }

    /// <summary>The aux tuning view.</summary>
    [Serializable]
    public sealed class CSceneVehicleCarTuningAux
    {
        public float steeringSpeedBase;          /* +0x06c */
        public float steeringSpeedScale;         /* +0x070 */
        public float steeringSlewRate;           /* +0x094 */
        public float airTorqueLinear;            /* +0x158 */
        public float airTorqueQuadratic;         /* +0x15c */
        public float suspensionFollowRate;       /* +0x194 */
        public uint airControlWindowTicks;       /* +0x364 */
        public float airReversalThreshold;       /* +0x368 */
        public CFuncKeysReal airVerticalCurve;   /* +0x36c */
        public CFuncKeysReal steeringAngleCurve; /* +0x378 */
        public float waterBuoyancy;              /* +0x204 */
        public float waterEntrySpeedThreshold;   /* +0x208 */
        public float waterEntrySpeedMinimum;     /* +0x20c */
        public CFuncKeysReal waterImpulseVerticalCurve;   /* +0x210 */
        public CFuncKeysReal waterImpulseHorizontalCurve; /* +0x214 */
        public CFuncKeysReal waterFrictionCurve;          /* +0x218 */
        public float waterAngularDragLinear;     /* +0x21c */
        public float waterAngularDragQuadratic;  /* +0x220 */
    }

    /// <summary>CSceneVehicleMaterialBlendableVals (0x10 bytes).</summary>
    [Serializable]
    public struct CSceneVehicleMaterialBlendableVals
    {
        public float acceleration;
        public float braking;
        public float steering;
        public float lateralGrip;
    }

    /// <summary>Per-ground-id material: blend values + bump descriptor.</summary>
    [Serializable]
    public sealed class TMNFVehicleGroundMaterial
    {
        public float[] values = new float[4];
        /// <summary>The shared 128x128 bump image (null = no bumps).</summary>
        public byte[] fakeContactMask;
        public float fakeContactPeriodX;
        public float fakeContactPeriodZ;
        public float fakeContactImpulseScale;
        public float fakeContactImpulseLimit;
    }

    /// <summary>TMNFVehicleContactTuning.</summary>
    [Serializable]
    public sealed class TMNFVehicleContactTuning
    {
        public float frictionForce;
        public float extraFrictionForce;
        public float slopeAdherenceMin;
        public float slopeAdherenceMax;
        public float slopeSecondaryMin;
        public float slopeSecondaryMax;
        public float angularYScale;
        public float angularXzScale;
        public float damperMax;
        public float maxAngularSpeed;
        public float maxLinearSpeedDelta;
        public float bodyTangentRatio;
        public float bodyTangentRatioMaterial4;
        public float restitutionAirMaterial4;
        public float restitutionAir;
        public float restitutionGround;
        public float restitutionGroundMaterial4;
        public float lateralLinear;
        public float lateralQuadratic;
        public float lateralGroundScale;
        public uint lateralContactDurationTicks;
        public int wheelContactModel;
        public int frictionModel;
    }

    /// <summary>TMNFVehicleComputeTuning.</summary>
    [Serializable]
    public sealed class TMNFVehicleComputeTuning
    {
        public float eventCLevel1Max;            /* +0x028 */
        public float groundedDragTerm;           /* +0x058 */
        public float activeContactStopThreshold; /* +0x0a4 */
        public float normalTurboFactor;          /* +0x0f0 */
        public float rouletteTurboFactor;        /* +0x0f4 */
        public uint normalTurboDuration;         /* +0x0f8 */
        public uint rouletteTurboDuration;       /* +0x0fc */
        public float airImpulseScale;            /* +0x104 */
        public float specialForceFieldScale;     /* +0x108 */
        public float airborneLinearDrag;         /* +0x154 */
        public float groundedForceFieldScale;    /* +0x160 */
        public float airborneForceFieldScale;    /* +0x164 */
        public float normalizedForceDivisor;     /* +0x228 */
        public CFuncKeysReal effectCurve;        /* +0x380 */
        public float effectCurveBias;            /* +0x384 */
        public float eventAbLevel2Min;           /* +0x398 */
        public float eventAbTrigger;             /* +0x39c */
        public float eventCTrigger;              /* +0x3a0 */
        public CFuncKeysReal contactDecayCurve;  /* primary +0x044 */
        public CFuncKeysReal contactRiseCurve;   /* primary +0x048 */
    }

    /// <summary>The model-6 scalar view.</summary>
    [Serializable]
    public sealed class CSceneVehicleCarModel6Tuning
    {
        public float forwardSpeedLimitScale;
        public float reverseSpeedLimitScale;
        public float brakeBase;
        public float brakeSpeedScale;
        public float forwardBrakeLimitSliding;
        public float forwardBrakeLimit;
        public float speedLimitForce;
        public float verticalForceScale;
        public float wheelSteerSineLimit;
        public float steerSlowdownScale;
        public float wheelTorqueScale;
        public float slidingSteerTorqueScale;
        public float lateralForceScale;
        public float slidingLateralLimitScale;
        public float lateralOverflowBlend;
        public float wheelOverflowBlend;
        public float verticalForceDivisor;
        public float tractionLossScale;
        public float burnoutTriggerScale;
        public float burnoutTriggerLimit;
        public float rolloverAxisMinLength;
        public float rolloverTorqueXScale;
        public float rolloverTorqueZScale;
        public float slidingBrakeScale;
        public float brakingLateralLimitScale;
        public float reverseBrakeLimitSliding;
        public float reverseBrakeLimit;
        public float burnoutSpeedMax;
        public float burnoutSpeedMin;
        public float donutLateralForceScale;
        public float donutYawAngleScale;
        public float donutSteerLinear;
        public float donutSteerQuadratic;
        public float donutCountersteerScale;
        public float donutRadiusExponent;
        public float donutRadialSpeedExponent;
        public float donutRadiusMin;
        public float donutLateralSpeedLimit;
        public float donutNormalAngleLimit;
        public float donutAnglePositiveLimit;
        public float donutAngleNegativeLimit;
        public uint burnoutEnterTicks;
        public float burnoutEnterAccelScale;
        public float burnoutEnterLateralScale;
        public uint burnoutExitTicks;
        public float burnoutExitAccelScale;
        public float burnoutExitExtraAccel;
        public float material6LongitudinalScale;
        public float material6GasDenominator;
        public float material6VerticalShape;
        public float material6VerticalScale;
    }

    /// <summary>The complete immutable tuning bundle.</summary>
    [Serializable]
    public sealed class VehicleTuning
    {
        public CSceneVehicleCarTuning @base = new CSceneVehicleCarTuning();
        public CSceneVehicleCarTuningAux aux = new CSceneVehicleCarTuningAux();
        public TMNFVehicleContactTuning contact = new TMNFVehicleContactTuning();
        public TMNFVehicleComputeTuning compute = new TMNFVehicleComputeTuning();
        public CSceneVehicleCarModel6Tuning model6 = new CSceneVehicleCarModel6Tuning();
        public CSceneVehicleCarTuningCurveSet curves = new CSceneVehicleCarTuningCurveSet();
        /// <summary>31-entry ground-id table indexing groundMaterials.</summary>
        public List<uint> groundMaterialIndices = new List<uint>();
        public List<TMNFVehicleGroundMaterial> groundMaterials = new List<TMNFVehicleGroundMaterial>();
        /// <summary>object_refs of the four wheel collision trees.</summary>
        public List<uint> wheelTreeRefs = new List<uint>();
        /// <summary>object_refs of the body collision trees.</summary>
        public List<uint> bodyTreeRefs = new List<uint>();

        public VehicleTuning Clone()
        {
            return (VehicleTuning)MemberwiseClone();
        }
    }

    /// <summary>0x30 bytes; the engine state embedded at car+0x59C.</summary>
    [Serializable]
    public struct CSceneVehicleCarEngine
    {
        public float maxRpm;          /* 0x00 */
        public float brakingFactor;   /* 0x14 */
        public float rpm;             /* 0x18 */
        public float targetRpm;       /* 0x1c */
        public float clutch;          /* 0x20 */
        public float shiftTimer;      /* 0x24 */
        public int reverse;           /* 0x28 */
        public int gear;              /* 0x2c */
    }

    /// <summary>0xa8 bytes; the real-time wheel state at wheel+0xB4.</summary>
    [Serializable]
    public struct CSceneVehicleCarWheelRealTimeState
    {
        public float damperAbsorb;       /* 0x00 */
        public float field04;
        public float field08;
        public GmMat3 basis0;            /* 0x0c */
        public GmMat3 basis1;            /* 0x30 */
        public GmVec3 field54;           /* 0x54 */
        public byte[] reserved60;
        public float field6c;
        public int hasGroundContact;     /* 0x70 */
        public int contactMaterialId;    /* 0x74 */
        public int isSliding;            /* 0x78 */
        public GmVec3 relativeRotzAxis;  /* 0x7c */
        /// <summary>Game 32-bit pointer word (provenance only).</summary>
        public uint contactBodyWord;     /* 0x88 */
        public int groundContactCount;   /* 0x8c */
        public GmVec3 field90;
        public float rotationPhase;      /* 0x9c */
        public float blendValue;         /* 0xa0 */
        public float blendTarget;        /* 0xa4 */

        public static CSceneVehicleCarWheelRealTimeState Create()
        {
            return default(CSceneVehicleCarWheelRealTimeState);
        }
    }

    /// <summary>0x64 bytes of opaque per-history-slot wheel state.</summary>
    [Serializable]
    public struct CSceneVehicleCarWheelState
    {
        public byte[] bytes;

        public static CSceneVehicleCarWheelState Create()
        {
            CSceneVehicleCarWheelState s = default(CSceneVehicleCarWheelState);
            s.bytes = new byte[0x64];
            return s;
        }
    }

    /// <summary>0x14 bytes; the damped scalar spring.</summary>
    [Serializable]
    public struct GmSpringFloat
    {
        public float stiffness; /* 0x00 */
        public float damping;   /* 0x04 */
        public float value;     /* 0x08 */
        public float target;    /* 0x0c */
        public float velocity;  /* 0x10 */
    }

    /// <summary>Turbo type.</summary>
    public enum TMNFVehicleTurboType
    {
        None = 0,
        Normal = 1,
        Roulette = 2,
    }

    /// <summary>Model-6 state block (car offsets in the comments).</summary>
    [Serializable]
    public sealed class CSceneVehicleCarModel6State
    {
        public GmVec3 pivotPosition;       /* +1dc */
        public GmVec3 pivotAxis;           /* +1e8 */
        public int reverseMode;            /* +5c4 */
        public float reverseSpeedThreshold;/* +5cc */
        public int contactBlockCount;      /* +5d8 */
        public int sideContact;            /* +5dc */
        public uint lastSlidingTick;       /* +62c */
        public uint slidingStartTick;      /* +630 */
        public uint slidingElapsedTicks;   /* +634 */
        public GmIso4 modelIso;            /* +6a4 */
        public GmVec3 rolloverAxis;        /* +6d4 */
        public GmVec3 orbitCenter;         /* +6e0 */
        public float orbitInitialRadius;   /* +6ec */
        public float orbitRadius;          /* +6f0 */
        public uint burnoutStartTick;      /* +6f4 */
        public uint burnoutTransitionTick; /* +6f8 */
        public GmVec3 orbitAxis;           /* +6fc */
        public float orbitSign;            /* +708 */
        public float axleWidth;            /* +840 */

        public static CSceneVehicleCarModel6State Create()
        {
            CSceneVehicleCarModel6State s = new CSceneVehicleCarModel6State();
            s.modelIso = GmIso4.Identity;
            return s;
        }
    }

    /// <summary>TMNFVehicleComputeState (car fields used by the force caller).</summary>
    [Serializable]
    public sealed class TMNFVehicleComputeState
    {
        public float simulationGate;              /* car +0x1e8 */
        public uint eventLevelC;                  /* car +0x1fc */
        public byte eventSourceC;                 /* car +0x200 */
        public byte eventSourceAb;                /* car +0x201 */
        public GmSpringFloat springC;             /* car +0x214 */
        public GmSpringFloat springA;             /* car +0x228 */
        public float contactRise;                 /* car +0x23c */
        public float contactDecay;                /* car +0x240 */
        public float historyForceLimit;           /* car +0x244 */
        public float historyForceScale;           /* car +0x24c */
        public float springValueLimit;            /* car +0x250 */
        public float localSpeedLimit;             /* car +0x2e0 */
        public float airEffectThreshold;          /* car +0x05c */
        public float brakeInputScale;             /* car +0x5a8 */
        public float groundedDragScale;           /* car +0x5ac */
        public float computedBrakeForce;          /* car +0x5b0 */
        public int state5d8;                      /* car +0x5d8 */
        public uint airImpulseCooldownTick;       /* car +0x610 */
        public float effectAccumulator;           /* car +0x624 */
        public uint lastForceTick;                /* car +0x650 */
        public uint eventLevelA;                  /* car +0x654 */
        public uint eventLevelB;                  /* car +0x658 */
        public uint peakEventLevelB;              /* car +0x660 */
        public uint peakEventLevelA;              /* car +0x664 */
        public uint peakEventLevelC;              /* car +0x668 */
        public byte peakEventSourceAb;            /* car +0x66c */
        public byte peakEventSourceC;             /* car +0x66d */
        public float eventMetricA;                /* car +0x670 */
        public float eventMetricB;                /* car +0x674 */
        public float eventMetricC;                /* car +0x678 */
        public GmVec3 normalizedForce;            /* car +0x6d4 */
        public int airEffectMode;                 /* car +0x74c */
    }

    /// <summary>One wheel: the folded CSceneVehicleCarWheel (+aux +contact).</summary>
    [Serializable]
    public sealed class CSceneVehicleCarWheel
    {
        public int active;                    /* +0x000 */
        public int steerable;                 /* +0x004 */
        public float radius;                  /* +0x008 */
        public GmIso4 surfaceSource;          /* +0x010 (aux view) */
        public GmIso4 surfaceLocation;        /* +0x040 (aux view) */
        public float[] field70 = new float[12];/* +0x070..+0x0a0 */
        public int fielda0;
        public int fielda4;
        public GmVec3 offsetFromVehicle;      /* +0x0a8 */
        public CSceneVehicleCarWheelRealTimeState realTime = default(CSceneVehicleCarWheelRealTimeState); /* +0x0b4 */
        public int field15c;                  /* +0x15c (contact_relative_valid) */
        public GmVec3 contactRelativeLocalDistance; /* +0x160 */
        /// <summary>Wheel +0x64 impulse point (contact view's copy).</summary>
        public GmVec3 impulsePoint;
        /// <summary>Contacted static body by corpus ref (null = none).</summary>
        public uint? contactBody;
        // v6 snapshot history (400 bytes) — capture/restore/respawn only.
        public CSceneVehicleCarWheelState previousSync; /* +0x16c */
        public CSceneVehicleCarWheelState sync;         /* +0x1d0 */
        public CSceneVehicleCarWheelState field234;     /* +0x234 */
        public CSceneVehicleCarWheelState asyncState;   /* +0x298 */

        public CSceneVehicleCarWheel()
        {
            surfaceSource = GmIso4.Identity;
            surfaceLocation = GmIso4.Identity;
            previousSync = CSceneVehicleCarWheelState.Create();
            sync = CSceneVehicleCarWheelState.Create();
            field234 = CSceneVehicleCarWheelState.Create();
            asyncState = CSceneVehicleCarWheelState.Create();
            realTime.reserved60 = new byte[0x0c];
        }
    }

    /// <summary>ALL mutable per-simulation vehicle state.</summary>
    [Serializable]
    public sealed class Vehicle
    {
        // --- CSceneVehicleCar ---
        public float inputGas;            /* +0x050 */
        public float inputBrake;          /* +0x054 */
        public float inputSteer;          /* +0x058 */
        public List<CSceneVehicleCarWheel> wheels = new List<CSceneVehicleCarWheel>();
        public uint wheelCount;           /* +0x2ec */
        public CSceneVehicleCarEngine engine; /* +0x59c */
        public GmVec3 currentLocalSpeed; /* +0x70c */
        public GmVec3 totalForceAdded;   /* +0x818 */
        public GmVec3 totalImpulseAdded; /* +0x824 */
        public int engineMode;           /* +0x2e4 */
        public int turboActive;          /* +0x628 */
        public int driveMode;            /* +0x69c */
        public int engineLimitFlag;      /* +0x744 */
        public int gearDownshiftFlag;    /* +0x748 */
        public int forceWheelSpeed;      /* +0x6a0 */
        public int flag60c;              /* +0x60c */
        public int blockWheelSpeed;      /* +0x73c */
        public float forcedWheelSpeed;   /* tuning-selected source +0x2bc */

        // --- aux state ---
        public uint integrationFlags;    /* +0x2f4 */
        public uint turboEpochTick;      /* +0x5d0 */
        public int airControlImmediate;  /* +0x5d4 */
        public int airControlLocked;     /* +0x5e4 */
        public float steeringValue;      /* +0x5e8 */
        public float turboProgress;      /* +0x5f0 */
        public float turboFactor;        /* +0x5f4 */
        public uint turboStartTick;      /* +0x5f8 */
        public uint turboEndTick;        /* +0x5fc */
        public TMNFVehicleTurboType turboType = TMNFVehicleTurboType.None; /* +0x600 */
        public uint rouletteToken;       /* +0x604 */
        public float rouletteValue;      /* +0x608 */
        public uint airControlTick;      /* +0x614 */
        public GmVec3 airControlSpeed;   /* +0x618 */
        /// <summary>Runtime DAT_00d06a74.</summary>
        public uint rouletteModulus;
        /// <summary>Car +0x1dc..+0x1f4: the body box (pivot words).</summary>
        public GmBoxAligned bodyBox;
        /// <summary>Game car +0x26c != NULL.</summary>
        public bool turboSoundAttached;

        // --- contact state ---
        public int frictionInputSelector; /* +0x5c4 */
        public int sideContact;           /* +0x5dc */
        public uint lastSideContactTick;  /* +0x5e0 */
        public int airborneFrictionGate;  /* +0x5e4 */
        public uint wheelContactAbsorbCount;
        public uint bodyContactCount;
        public GmVec3 bodyContactPositionSum;
        public GmVec3 bodyContactNormalSum;
        /// <summary>TMNFVehicleContactTimer (CMwTimerAdapter tick).</summary>
        public uint tickTime;

        // --- model 6 + compute state ---
        public CSceneVehicleCarModel6State model6 = CSceneVehicleCarModel6State.Create();
        public TMNFVehicleComputeState compute = new TMNFVehicleComputeState();

        public Vehicle()
        {
            bodyBox = default(GmBoxAligned);
        }
    }

    /// <summary>
    /// The bundle passed where the original passed one of its context
    /// structs: the vehicle, its tuning and the dyna travel together.
    /// </summary>
    public struct VehicleCtx
    {
        public Vehicle vehicle;
        public VehicleTuning tuning;
        public CHmsDyna dyna;

        public VehicleCtx(Vehicle vehicle, VehicleTuning tuning, CHmsDyna dyna)
        {
            this.vehicle = vehicle;
            this.tuning = tuning;
            this.dyna = dyna;
        }

        public ref CHmsStateDyna Live() => ref dyna.Live();

        public CHmsDynaParams Params => dyna.@params;

        /// <summary>A by-ref view of the dyna params (for in-place reads).</summary>
        public ref CHmsDynaParams DynaParams() => ref dyna.@params;
    }

    /// <summary>The world-supplied externals for model-6 entry.</summary>
    [Serializable]
    public struct Model6External
    {
        public GmIso4 modelIsoSource;
        public GmVec3 bodyReferencePosition;
    }
}
