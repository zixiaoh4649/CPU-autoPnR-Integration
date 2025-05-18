# proc is a function. this is used later to connect all the vdd! and vss! together.
proc connectGlobalNets {} {
	globalNetConnect vdd! -type pgpin -pin vdd! -all
	globalNetConnect vss! -type pgpin -pin vss! -all
	globalNetConnect vdd! -type tiehi -all
	globalNetConnect vss! -type tielo -all
	applyGlobalNets
}

# set the top level module name (used elsewhere in the scripts)
set design_toplevel cpu

# set the verilog file to pnr
set init_verilog ../synth/outputs/$design_toplevel.v

# set the lef file of your standard cells
# when you add your regfile lef, it is here
# if you want to supply more than one lef use the following syntax:
# set init_lef_file "1.lef 2.lef"
set init_lef_file "../stdcells.lef ../regfile.lef"

# actually set the top level cell name
set init_top_cell $design_toplevel

# set power and ground net names
set init_pwr_net vdd!
set init_gnd_net vss!

# set multi-mode multi-corner file
# this file contains the operating conditions used to evaluate timing
# for your design. In our case, we just use the single lib file as our corner.
# In ECE 498HK, this will contain slow, typical and fast corners
# for the wires and the standard cells
set init_mmmc_file mmmc.tcl

# actually init the design
init_design

# connect all the global nets in the design together (vdd!, vss!)
# the function is defined above.
connectGlobalNets

# TODO floorplan your design. Put the size of your chip that you want here.
# floorPlan -site CoreSite -s $TODO $TODO 10 10 10 10
set die_w 190
set die_h 130
set m_left   10
set m_bottom 10
set m_right  10
set m_top    10
floorPlan -site CoreSite \
          -d $die_w $die_h $m_left $m_bottom $m_right $m_top


# create the horizontal vdd! and vss! wires used by the standard cells.
sroute -allowJogging 0 -allowLayerChange 0 -crossoverViaLayerRange { metal7 metal1 } -layerChangeRange { metal7 metal1 } -nets { vss! vdd! }



# create a power ring around your processor, connecting all the vss! and vdd! together physically.
addRing \
	-follow core \
	-offset {top 2 bottom 2 left 2 right 2} \
	-spacing {top 2 bottom 2 left 2 right 2} \
	-width {top 2 bottom 2 left 2 right 2} \
	-layer {top metal7 bottom metal7 left metal8 right metal8} \
	-nets { vss! vdd! }

# TODO add power grid
addStripe \
   -nets           {vdd! vss!} \
   -direction      vertical \
   -layer          metal6\
   -width          1 \
   -spacing        1 \
   -set_to_set_distance 230\
   -start_from     left\
   -start_offset 4


# TODO restrict routing to only metal 6
# setDesignMode ...
setDesignMode \
    -bottomRoutingLayer metal1 \
    -topRoutingLayer    metal6
# TODO for the regfile part, place the regfile marco
# placeInstance datapath/bitslices[3].bitslice/regfile 10 10 R0



set x0      20.0
set y0      20.0
set pitch_y 1.185          ;

for {set i 0} {$i < 32} {incr i} {
 
    set inst [format {datapath/bitslices[%d].bitslice/regfile} $i]
    set y [expr {$y0 + $i * $pitch_y}]

    placeInstance $inst $x0 $y R0 
}


# TODO specify where are the pins
# editPin ...


set pinList [dbGet top.terms.name]
editPin \
   -side RIGHT\
   -layer metal4\
   -fixedPin 1\
   -spreadType RANGE \
   -start {0 0}\
   -end {0 0}\
   -spreadDirection CounterClockwise\
   -pin {\
    clk \
    rst \
    dmem_write \
    dmem_wmask[*] \
    imem_addr[*] \
   }

editPin \
   -side BOTTOM\
   -layer metal4\
   -fixedPin 1\
   -spreadType RANGE \
   -start {0 0}\
   -end {999 0}\
   -spreadDirection CounterClockwise\
   -pin {\

    imem_rdata[*] \

   }

editPin \
   -side TOP\
   -layer metal4\
   -fixedPin 1\
   -spreadType RANGE \
   -start {999 999}\
   -end {999 999}\
   -spreadDirection CounterClockwise\
   -pin {\
    dmem_addr[*] \
    dmem_rdata[*] \
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
    dmem_wdata[*] \

   }



# TODO uncomment the two below command to do pnr. These steps takes innovus more time.
place_design

routeDesign
# connectGlobalNets
connectGlobalNets         

# TODO find the command that checks DRC
verify_drc

# save your design as a GDSII, which you can open in Virtuoso
streamOut innovus.gdsii -mapFile "/class/ece425/innovus.map"

# save the design, so innovus can open it later
saveDesign $design_toplevel
