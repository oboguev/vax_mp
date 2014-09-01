$!
$!  LAUNCH.COM - launch specified number of stress subprocesses,
$!               then stop them all on Ctrl/Y
$!
$!  When running this test you may either run a scenario that exercises "soft" paging
$!  between process space and modified list, or "hard" paging with IO to/from paging file.
$!
$!  In the latter case you want to reduce the size both of physical memory avalable and of
$!  modified list.
$!
$!  Size of physical memory can be reduced either with SIMH configuration or with SYSGEN
$!  parameter PHYSICALPAGES.
$!
$!  To reduce modified-list size, reduce MPW_HILIMIT, MPW_LOLIMIT and associated
$!  MPW_WAITLIMIT and MPW_LOWAITLIMIT, for example:
$!
$!      USE CURRENT
$!      WR SAFE.PAR
$!      SET PHYSICALPAGES 30000
$!      SET MPW_LOLIMIT 50
$!      SET MPW_HILIMIT 300
$!      SET MPW_WAITLIMIT 300
$!      SET MPW_LOWAITLIMIT 200
$!      SET WSMAX 2048
$!      WR PAGETEST.PAR
$!
$!  then use SYSBOOT to start system with these parameters
$!  (remember to SET WRITESYS 0 when executing SYSBOOT!)
$!  and run:
$!
$!      @LAUNCH 20 2000
$!
$!  yelding demand of 40,000 memory pages exceeding free list and causing the pages
$!  to be both written to page file and read from it.
$!
$!  Note: We observed TCPIP services for OpenVMS crash system under tight memory
$!        configuration of 28,000 pages. Crash happened in 50-second periodic TQE
$!        queued by TCPIP (invalid TQE$L_FPC).
$!
$!SET VERIFY
$ TNAME = "ST_PAGE"
$ SAY := WRITE SYS$OUTPUT
$!
$ IF P1 .EQS. "" .OR. F$TYPE(P1) .NES. "INTEGER" THEN GOTO USAGE
$ NPROCS = F$INTEGER(P1)
$ IF NPROCS .LE. 0 .OR. NPROCS .GT. 99 THEN GOTO BAD_NPROCS
$!
$ IF P2 .EQS. "" .OR. F$TYPE(P2) .NES. "INTEGER" THEN GOTO USAGE
$ NPAGES = F$INTEGER(P2)
$ IF NPAGES .LE. 0 THEN GOTO BAD_NPAGES
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
$!GOSUB CLEAN
$!IF F$SEARCH("TEMP.DIR") .EQS. "" THEN CREATE/DIRECTORY/NOLOG [.TEMP]
$!
$!
$ NP = 1
$START_LOOP:
$ SPAWN /LOG/NOWAIT/PROCESS='TNAME'_'NP' @'RUN_FILE' 'NP' 'NPAGES'
$ NP = NP + 1
$ IF NP .LE. NPROCS THEN GOTO START_LOOP
$ SAY ""
$ SAY "*** Press CTRL/Y to terminate started processes ***"
$ SAY "*** Waiting for CTRL/Y to be pressed ..."
$ SAY ""
$!
$!
$WAIT_LOOP:
$ WAIT 20:00
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
$ SET PROCESS/PRIO=4 'TNAME'_'NP'
$ STOP 'TNAME'_'NP'
$NOPROCESS:
$ ON WARNING THEN EXIT
$ ON ERROR THEN EXIT
$ ON SEVERE_ERROR THEN EXIT
$ NP = NP + 1
$ IF NP .LE. NPROCS THEN GOTO STOP_LOOP
$!SAY "*** Removing temporary files ..."
$!SAY ""
$ WAIT 0:0:2
$!GOSUB CLEAN
$ SET PROCESS/PRIO='SV_PRIO'
$ SET DEFAULT 'SV_DEF'
$ EXIT
$!
$!
$USAGE:
$ SAY "Usage: @LAUNCH NPROCS NPAGES"
$ EXIT
$BAD_NPROCS:
$ SAY "NPROCS should be in 1 ... 99 range"
$ EXIT
$BAD_NPAGES:
$ SAY "NPAGES should be above 0"
$ EXIT
$!
$!
$!CLEAN:
$!IF F$SEARCH("[.TEMP.*]*.*;*") .NES. "" THEN DELETE/NOLOG [.TEMP.*]*.*;*
$!IF F$SEARCH("[.TEMP]*.*;*") .NES. "" THEN DELETE/NOLOG [.TEMP]*.*;*
$!IF F$SEARCH("TEMP.DIR;") .NES. "" THEN DELETE/NOLOG TEMP.DIR;*
$!RETURN