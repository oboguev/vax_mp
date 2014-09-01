    .TITLE      Error and Warning Messages for virtual SMP control utility
    .IDENT      /V-1.0/

    .FACILITY   VSMP,1819 /PREFIX=VSMP_MSG_

!
! Falal error messages
!
.SEVERITY FATAL
.BASE 1
    SYS_NOT_VAXMP         <Not running on a simulator with VAX MP interface>
    VMS_NOT_MULTI         <Loaded OpenVMS system image is uniprocessor>

.BASE 20
    MISLINKED             <This image is not properly linked, internal verification failed>
    MISBUILT              <This image is not properly built, internal verification failed>
    SYNTAX                <Invalid command syntax>
    ALREADY_SMP           <Multiprocessing is already active>
    ALREADY_LOADED        <VSMP resident image had been already loaded>
    NOT_LOADED            <VSMP resident image is not loaded>
    VERSION_MISMATCH      <Version of loaded VSMP resident image mismatches control utility version>
    CALIBR_UNSTABLE       <Unable to achieve stable calibration of system timing loops>
    LDR_VERIFY            <Unable to load VSMP resident image because pre-load verification failed>
    LDR_SCH_CUR_TO_COM    <Unexpected routine SCH$CUR_TO_COM is present in loaded system image>
    LDR_EXE_PROC_IDLE     <Unexpected state of vector for EXE$PROC_IDLE>
    LDR_SMP_JMPVEC        <Unexpected structure of SMP vectored routines>
    VM_REFUSED            <Virtual machine refused initiaton of SMP>
    IDLE_NEVER            <IDLE had been set to NEVER and cannot be changed>
    UNMOD_DRV             <Loaded unmodified device drivers prevent multiprocessor operation>
    ALLOCPFN              <Unable to allocate system memory pages for fork-to-primary pool>
    INVOPTVAL             <Invalid value for option !AZ>  /FAO=1
    IVPATCHID             <Invalid patch ID argument: !AZ>  /FAO=1
    XDTINVALID            <XDelta is loaded but is not in the valid state>
    MSGRTNSLDR            <Unable to locate loaded executive image MESSAGE_ROUTINES.EXE>
    XDELTA_P              <Unable to locate XDELTA patch area>
    CHSEP_P               <Unable to locate CHSEP patch area>
    RESCHED_P             <Unable to locate RESCHED patch area>
    NUMTIM_P              <Unable to locate NUMTIM patch area>
    MFYCAP_P              <Unable to locate MFYCAP (ADD_CAP/REMOVE_CAP) patch area>
    CRBTMO_P              <Unable to locate CRBTMO patch area>
    UCBTMO_P              <Unable to locate UCBTMO patch area>
    XQTIMXMT_P            <Unable to locate XQTIMXMT patch area>
    UNSUPPXQ              <VSMP currently supports only DEQNA and DELQA, not DEQTA>
    XQTIMXMT_C            <Unable to apply XQTIMXMT patch, consistency check failed>
    XQTX1_P               <Unable to locate XQTX1 patch area>
    XQTX2_P               <Unable to locate XQTX2 patch area>
    XQTX3_P               <Unable to locate XQTX3 patch area>
    XQTX4_P               <Unable to locate XQTX4 patch area>
    XQTX5_P               <Unable to locate XQTX5 patch area>
    XQTX6_P               <Unable to locate XQTX6 patch area>
    XQTX7_P               <Unable to locate XQTX7 patch area>
    XQTX8_P               <Unable to locate XQTX8 patch area>
    XQTX9_P               <Unable to locate XQTX9 patch area>
    XQTX10_P              <Unable to locate XQTX10 patch area>
    XQRX1_P               <Unable to locate XQRX1 patch area>
    XQRX2_P               <Unable to locate XQRX2 patch area>
    XQRX3_P               <Unable to locate XQRX3 patch area>
    XQRX4_P               <Unable to locate XQRX4 patch area>
    PU1_P                 <Unable to locate PU1 patch area>
    PU2_P                 <Unable to locate PU2 patch area>
    PU3_P                 <Unable to locate PU3 patch area>
    PU4_P                 <Unable to locate PU4 patch area>
    PU5_P                 <Unable to locate PU5 patch area>
    PU6_P                 <Unable to locate PU6 patch area>
    PU7_P                 <Unable to locate PU7 patch area>
    IVPARSET              <Invalid settings combination: !AZ>  /FAO=1
    IVLOCKRTRY            <Invalid value of SYSGEN parameter LOCKRETRY>
    VLOW_SW_SYS           <Very low size of SYS synchronization window>
    VLOW_SW_ILK           <Very low size of ILK synchronization window>
    UNSTART               <Unable to start multiprocessing>
    SPWHIGH               <Value of SYSGEN parameter SMP_SPINWAIT is too high>
    LSPWHIGH              <Value of SYSGEN parameter SMP_LNGSPINWAIT is too high>


!
! Informational messages
!
.SEVERITY INFORMATIONAL
.BASE 100

    CALIBRATING           <Calibrating OpenVMS timing loops...>
    CALIBRATED            <Calibration completed at !UL samples>  /FAO=1
    CALIBRETRY            <Calibration was unstable, trying one more pass...>
    LOADED                <VSMP resident image was loaded into the kernel and activated>
    ADV_SW_SYS            <Check values of SYSGEN parameters SMP_SPINWAIT, SMP_LNGSPINWAIT and VSMP option SYNCW_SYS>
    ADV_SW_ILK            <Check values of SYSGEN parameter LOCKRETRY and VSMP option SYNCW_ILK>
    ADV_ISW_SYS           <Advise to increase values of SYSGEN parameters SMP_SPINWAIT, SMP_LNGSPINWAIT>
    ADV_SPW_HIGH          <Reduce SMP_SPINWAIT to !UL and reboot> /FAO=1
    ADV_LSPW_HIGH         <Reduce SMP_LNGSPINWAIT to !UL and reboot> /FAO=1

!
! Warning messages
!
.SEVERITY WARNING
.BASE 200

    NONATIVE              <Native interlock is not available, falling back to portable>
    SYNCWIDLEOFF          <IDLE changed from NEVER to OFF because SYNCW=ILK is enabled>
    LOW_SW_SYS            <Low size of SYS synchronization window>
    LOW_SW_ILK            <Low size of ILK synchronization window>
    SYS_LESS_ILK          <Synchronization window SYS is tighter than ILK window>
    .END
