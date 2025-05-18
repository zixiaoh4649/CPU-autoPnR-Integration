proc connectGlobalNets {} {
	globalNetConnect vdd! -type pgpin -pin vdd! -all
	globalNetConnect vss! -type pgpin -pin vss! -all
	globalNetConnect vdd! -type tiehi -all
	globalNetConnect vss! -type tielo -all
	applyGlobalNets
}

set design_toplevel control
set init_verilog ../synth/outputs/$design_toplevel.v
set init_lef_file ../stdcells.lef
set init_top_cell $design_toplevel
set init_pwr_net vdd!
set init_gnd_net vss!
set init_mmmc_file mmmc.tcl

init_design
connectGlobalNets


set die_w 55
set die_h 35
set m_left   10
set m_bottom 10
set m_right  10
set m_top    10
floorPlan -site CoreSite \
          -d $die_w $die_h $m_left $m_bottom $m_right $m_top


sroute -allowJogging 0 -allowLayerChange 0 -crossoverViaLayerRange { metal7 metal1 } -layerChangeRange { metal7 metal1 } -nets { vss! vdd! }

addRing \
	-follow core \
	-offset {top 2 bottom 2 left 2 right 2} \
	-spacing {top 2 bottom 2 left 2 right 2} \
	-width {top 2 bottom 2 left 2 right 2} \
	-layer {top metal7 bottom metal7 left metal8 right metal8} \
	-nets { vss! vdd! }

addStripe \
   -nets           {vdd! vss!} \
   -direction      vertical \
   -layer          metal6\
   -width          1 \
   -spacing        1 \
   -set_to_set_distance 50\
   -start_from     left



setDesignMode \
    -bottomRoutingLayer metal1 \
    -topRoutingLayer    metal6


set pinList [dbGet top.terms.name]
editPin \
   -side BOTTOM\
   -layer metal6\
   -fixedPin 1\
   -spreadType RANGE \
   -start {0 0}\
   -end {15 0}\
   -spreadDirection CounterClockwise\
   -pin {\
    clk \
    dmem_write \
    mem_mux_sel[*] \
    rd_mux_sel[*] \
    pc_mux_sel \
    alu_mux_1_sel \
    alu_mux_2_sel \
    alu_inv_rs2 \
    alu_cin \
    alu_op[*] \
    shift_msb \
    shift_dir \
    cmp_mux_sel \
    cmp_out \
    cmp_lt \
    cmp_eq \
    cmp_a_31 \
    cmp_b_31 \
   }



editPin \
   -side LEFT\
   -layer metal4\
   -fixedPin 1\
   -spreadType RANGE \
   -start {0 0}\
   -end {0 0}\
   -spreadDirection CounterClockwise\
   -pin {\
   imem_rdata[*] \
   dmem_wmask[*] \
   }

editPin \
   -side LEFT\
   -layer metal3\
   -fixedPin 1\
   -spreadType RANGE \
   -start {0 0}\
   -end {0 0}\
   -spreadDirection CounterClockwise\
   -pin {\
    imm[*] \

   }


editPin -pin {rs2_sel[0]}  -side bottom -layer metal5 -assign {54.8.0000 0}        -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[1]}  -side bottom -layer metal5 -assign {52.6875 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[2]}  -side bottom -layer metal5 -assign {50.3750 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[3]}  -side bottom -layer metal5 -assign {48.0625 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[4]}  -side bottom -layer metal5 -assign {45.7500 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[5]}  -side bottom -layer metal5 -assign {43.4375 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[6]}  -side bottom -layer metal5 -assign {41.1250 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[7]}  -side bottom -layer metal5 -assign {38.8125 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[8]}  -side bottom -layer metal5 -assign {36.5000 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[9]}  -side bottom -layer metal5 -assign {34.1875 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[10]} -side bottom -layer metal5 -assign {31.8750 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[11]} -side bottom -layer metal5 -assign {29.5625 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[12]} -side bottom -layer metal5 -assign {27.2500 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[13]} -side bottom -layer metal5 -assign {24.9375 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[14]} -side bottom -layer metal5 -assign {22.6250 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[15]} -side bottom -layer metal5 -assign {20.3125 0}       -global_location -fixedPin -snap MGRID

editPin -pin {rs1_sel[0]}  -side bottom -layer metal5 -assign {54 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[1]}  -side bottom -layer metal5 -assign {52.3975 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[2]}  -side bottom -layer metal5 -assign {50.0850 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[3]}  -side bottom -layer metal5 -assign {47.7725 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[4]}  -side bottom -layer metal5 -assign {45.4600 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[5]}  -side bottom -layer metal5 -assign {43.1475 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[6]}  -side bottom -layer metal5 -assign {40.8350 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[7]}  -side bottom -layer metal5 -assign {38.5225 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[8]}  -side bottom -layer metal5 -assign {36.2100 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[9]}  -side bottom -layer metal5 -assign {33.8975 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[10]} -side bottom -layer metal5 -assign {31.5850 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[11]} -side bottom -layer metal5 -assign {29.2725 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[12]} -side bottom -layer metal5 -assign {26.9600 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[13]} -side bottom -layer metal5 -assign {24.6475 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[14]} -side bottom -layer metal5 -assign {22.3350 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[15]} -side bottom -layer metal5 -assign {20.0225 0}       -global_location -fixedPin -snap MGRID

editPin -pin {rd_sel[0]}  -side bottom -layer metal5 -assign {53.0750 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[1]}  -side bottom -layer metal5 -assign {50.7625 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[2]}  -side bottom -layer metal5 -assign {48.4500 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[3]}  -side bottom -layer metal5 -assign {46.1375 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[4]}  -side bottom -layer metal5 -assign {43.8250 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[5]}  -side bottom -layer metal5 -assign {41.5125 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[6]}  -side bottom -layer metal5 -assign {39.2000 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[7]}  -side bottom -layer metal5 -assign {36.8875 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[8]}  -side bottom -layer metal5 -assign {34.5750 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[9]}  -side bottom -layer metal5 -assign {32.2625 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[10]} -side bottom -layer metal5 -assign {29.9500 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[11]} -side bottom -layer metal5 -assign {27.6375 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[12]} -side bottom -layer metal5 -assign {25.3250 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[13]} -side bottom -layer metal5 -assign {23.0125 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[14]} -side bottom -layer metal5 -assign {20.7000 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[15]} -side bottom -layer metal5 -assign {18.3875 0}       -global_location -fixedPin -snap MGRID

editPin -pin {rd_sel[16]} -side bottom -layer metal6 -assign {53.3625 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[17]} -side bottom -layer metal6 -assign {51.0500 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[18]} -side bottom -layer metal6 -assign {48.7375 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[19]} -side bottom -layer metal6 -assign {46.4250 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[20]} -side bottom -layer metal6 -assign {44.1125 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[21]} -side bottom -layer metal6 -assign {41.8000 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[22]} -side bottom -layer metal6 -assign {39.4875 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[23]} -side bottom -layer metal6 -assign {37.1750 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[24]} -side bottom -layer metal6 -assign {34.8625 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[25]} -side bottom -layer metal6 -assign {32.5500 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[26]} -side bottom -layer metal6 -assign {30.2375 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[27]} -side bottom -layer metal6 -assign {27.9250 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[28]} -side bottom -layer metal6 -assign {25.6125 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[29]} -side bottom -layer metal6 -assign {23.3000 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[30]} -side bottom -layer metal6 -assign {20.9875 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rd_sel[31]} -side bottom -layer metal6 -assign {18.6750 0}       -global_location -fixedPin -snap MGRID

editPin -pin {rs1_sel[16]} -side bottom -layer metal4 -assign {54.1325 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[17]} -side bottom -layer metal4 -assign {51.8200 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[18]} -side bottom -layer metal4 -assign {49.5075 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[19]} -side bottom -layer metal4 -assign {47.1950 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[20]} -side bottom -layer metal4 -assign {44.8825 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[21]} -side bottom -layer metal4 -assign {42.5700 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[22]} -side bottom -layer metal4 -assign {40.2575 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[23]} -side bottom -layer metal4 -assign {37.9450 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[24]} -side bottom -layer metal4 -assign {35.6325 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[25]} -side bottom -layer metal4 -assign {33.3200 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[26]} -side bottom -layer metal4 -assign {31.0075 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[27]} -side bottom -layer metal4 -assign {28.6950 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[28]} -side bottom -layer metal4 -assign {26.3825 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[29]} -side bottom -layer metal4 -assign {24.0700 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[30]} -side bottom -layer metal4 -assign {21.7575 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs1_sel[31]} -side bottom -layer metal4 -assign {19.4450 0}       -global_location -fixedPin -snap MGRID

editPin -pin {rs2_sel[16]} -side bottom -layer metal4 -assign {53.0750 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[17]} -side bottom -layer metal4 -assign {50.7625 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[18]} -side bottom -layer metal4 -assign {48.4500 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[19]} -side bottom -layer metal4 -assign {46.1375 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[20]} -side bottom -layer metal4 -assign {43.8250 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[21]} -side bottom -layer metal4 -assign {41.5125 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[22]} -side bottom -layer metal4 -assign {39.2000 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[23]} -side bottom -layer metal4 -assign {36.8875 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[24]} -side bottom -layer metal4 -assign {34.5750 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[25]} -side bottom -layer metal4 -assign {32.2625 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[26]} -side bottom -layer metal4 -assign {29.9500 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[27]} -side bottom -layer metal4 -assign {27.6375 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[28]} -side bottom -layer metal4 -assign {25.3250 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[29]} -side bottom -layer metal4 -assign {23.0125 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[30]} -side bottom -layer metal4 -assign {20.7000 0}       -global_location -fixedPin -snap MGRID
editPin -pin {rs2_sel[31]} -side bottom -layer metal4 -assign {18.3875 0}       -global_location -fixedPin -snap MGRID



place_design

routeDesign

connectGlobalNets         
verify_drc

streamOut innovus.gdsii -mapFile "/class/ece425/innovus.map"
saveDesign $design_toplevel
