$!
$!  LAUNCH.COM - launch specified number of stress subprocesses,
$!               then stop them all on Ctrl/Y
$!
$!SET VERIFY
$ TNAME = "ST_FILE"
$ SAY := WRITE SYS$OUTPUT
$!
$ IF P1 .EQS. "" .OR. F$TYPE(P1) .NES. "INTEGER" THEN GOTO USAGE
$ NPROCS = F$INTEGER(P1)
$ IF NPROCS .LE. 0 .OR. NPROCS .GT. 99 THEN GOTO BAD_NPROCS
$!
$ IF (P2 .NES. "RMS") .AND. (P2 .NES. "QIO") THEN GOTO USAGE
$ MODE = P2
$!
$ INST_ID = P3
$!
$ IF P4 .NES. "" .AND. F$TYPE(P4) .NES. "INTEGER" THEN GOTO USAGE
$ NPASSES = P4
$!
$ PROC_FILE = F$ENVIRONMENT("PROCEDURE")
$ TEST_DIR = F$PARSE(PROC_FILE,,,"DEVICE","SYNTAX_ONLY") + -
             F$PARSE(PROC_FILE,,,"DIRECTORY","SYNTAX_ONLY")
$ RUN_FILE = TEST_DIR + "RUN.COM"
$ SV_PRIO = F$GETJPI("", "PRIB")
$ SV_DEF = F$ENVIRONMENT("DEFAULT")
$ SAY ""
$ SAY "*** Starting ''NPROCS' ''TNAME' processes ..."
$ SAY ""
$ ON CONTROL_Y THEN GOTO ON_CTRL_Y
$ SET PROCESS/PRIO=8
$ SET DEFAULT 'TEST_DIR'
$ GOSUB CLEAN
$ IF F$SEARCH("TEMP.DIR") .EQS. "" THEN CREATE/DIRECTORY/NOLOG [.TEMP]
$!
$!
$ NP = 1
$START_LOOP:
$ SPAWN /LOG/NOWAIT/PROCESS='TNAME'_'INST_ID''NP' @'RUN_FILE' 'MODE' 'NP' 'NPASSES'
$ NP = NP + 1
$ IF NP .LE. NPROCS THEN GOTO START_LOOP
$ SAY ""
$ SAY "*** Press CTRL/Y to terminate started processes ***"
$ SAY "*** Waiting for CTRL/Y to be pressed ..."
$ SAY ""
$!
$!
$WAIT_LOOP:
$ WRITE SYS$OUTPUT "    Timestamp: " + F$TIME()
$ WAIT 1:00:00
$ GOTO WAIT_LOOP
$!
$!
$ON_CTRL_Y:
$ SAY "*** Terminating ''TNAME' processes ..."
$ NP = 1
$STOP_LOOP:
$ ON WARNING THEN GOTO NOPROCESS
$ ON ERROR THEN GOTO NOPROCESS
$ ON SEVERE_ERROR THEN GOTO NOPROCESS
$ SET PROCESS/PRIO=4 'TNAME'_'INST_ID''NP'
$ STOP 'TNAME'_'INST_ID''NP'
$NOPROCESS:
$ ON WARNING THEN EXIT
$ ON ERROR THEN EXIT
$ ON SEVERE_ERROR THEN EXIT
$ NP = NP + 1
$ IF NP .LE. NPROCS THEN GOTO STOP_LOOP
$ SAY "*** Removing temporary files ..."
$ SAY ""
$ WAIT 0:0:2
$ GOSUB CLEAN
$ SET PROCESS/PRIO='SV_PRIO'
$ SET DEFAULT 'SV_DEF'
$ EXIT
$!
$!
$USAGE:
$ SAY "Usage: @LAUNCH NPROCS {RMS|QIO} [INST_ID] [NPASSES]"
$ SAY ""
$ SAY "    INST_ID can be A-Z and is used for naming processes, so multiple instances"
$ SAY "    of the test could run in parallel on different disk devices."
$ EXIT
$BAD_NPROCS:
$ SAY "NPROCS should be in 1 ... 99 range"
$ EXIT
$!
$!
$CLEAN:
$ IF F$SEARCH("[.TEMP.*]*.*;*") .NES. "" THEN DELETE/NOLOG [.TEMP.*]*.*;*
$ IF F$SEARCH("[.TEMP]*.*;*") .NES. "" THEN DELETE/NOLOG [.TEMP]*.*;*
$ IF F$SEARCH("TEMP.DIR;") .NES. "" THEN DELETE/NOLOG TEMP.DIR;*
$ RETURN