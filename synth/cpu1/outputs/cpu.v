/////////////////////////////////////////////////////////////
// Created by: Synopsys DC Expert(TM) in wire load mode
// Version   : W-2024.09
// Date      : Mon May  5 19:22:49 2025
/////////////////////////////////////////////////////////////


module control ( clk, imem_rdata, dmem_write, dmem_wmask, rs1_sel, rs2_sel, 
        rd_sel, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, alu_cin, alu_op, 
        shift_msb, shift_dir, cmp_mux_sel, pc_mux_sel, mem_mux_sel, rd_mux_sel, 
        cmp_out, imm, cmp_lt, cmp_eq, cmp_a_31, cmp_b_31 );
  input [31:0] imem_rdata;
  output [3:0] dmem_wmask;
  output [31:0] rs1_sel;
  output [31:0] rs2_sel;
  output [31:0] rd_sel;
  output [1:0] alu_op;
  output [2:0] mem_mux_sel;
  output [2:0] rd_mux_sel;
  output [31:0] imm;
  input clk, cmp_lt, cmp_eq, cmp_a_31, cmp_b_31;
  output dmem_write, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, alu_cin,
         shift_msb, shift_dir, cmp_mux_sel, pc_mux_sel, cmp_out;
  wire   alu_inv_rs2, N203, N204, N205, n1, n2, n3, n4, n5, n6, n7, n8, n9,
         n10, n11, n12, n13, n14, n15, n16, n17, n18, n19, n20, n21, n22, n23,
         n24, n25, n26, n27, n28, n29, n30, n31, n32, n33, n34, n35, n36, n37,
         n38, n39, n40, n41, n42, n43, n44, n45, n46, n47, n48, n49, n50, n51,
         n52, n53, n54, n55, n56, n57, n58, n59, n60, n61, n62, n63, n64, n65,
         n66, n67, n68, n69, n70, n71, n72, n73, n74, n75, n76, n77, n78, n79,
         n80, n81, n82, n83, n84, n85, n86, n87, n88, n89, n90, n91, n92, n93,
         n94, n95, n96, n97, n98, n99, n100, n101, n102, n103, n104, n105,
         n106, n107, n108, n109, n110, n111, n112, n113, n114, n115, n116,
         n117, n118, n119, n120, n121, n122, n123, n124, n125, n126, n127,
         n128, n129, n130, n131, n132, n133, n134, n135, n136, n137, n138,
         n139, n140, n141, n142, n143, n144, n145, n146, n147, n148, n149,
         n150, n151, n152, n153, n154, n155, n156, n157, n158, n159, n160,
         n161, n162, n163, n164, n165, n166, n167, n168, n169, n170, n171,
         n172, n173, n174, n175, n176, n177, n178, n179, n180, n181, n182,
         n183, n184, n185, n186, n187, n188, n189, n190, n191, n192, n193,
         n194, n195, n196, n197;
  assign imm[31] = imem_rdata[31];
  assign dmem_wmask[2] = dmem_wmask[3];
  assign alu_cin = alu_inv_rs2;
  assign mem_mux_sel[0] = N203;
  assign N203 = imem_rdata[12];
  assign mem_mux_sel[1] = N204;
  assign N204 = imem_rdata[13];
  assign mem_mux_sel[2] = N205;
  assign N205 = imem_rdata[14];

  inv U3 ( .A(clk), .Y(n1) );
  and2 U4 ( .A(n2), .B(imem_rdata[30]), .Y(shift_msb) );
  and2 U5 ( .A(cmp_a_31), .B(shift_dir), .Y(n2) );
  and2 U6 ( .A(n3), .B(n4), .Y(rs2_sel[9]) );
  and2 U7 ( .A(n5), .B(n3), .Y(rs2_sel[8]) );
  and2 U8 ( .A(n6), .B(n7), .Y(rs2_sel[7]) );
  and2 U9 ( .A(n8), .B(n6), .Y(rs2_sel[6]) );
  and2 U10 ( .A(n6), .B(n4), .Y(rs2_sel[5]) );
  and2 U11 ( .A(n6), .B(n5), .Y(rs2_sel[4]) );
  and2 U12 ( .A(n9), .B(n10), .Y(n6) );
  and2 U13 ( .A(n11), .B(n12), .Y(n9) );
  and2 U14 ( .A(n13), .B(n7), .Y(rs2_sel[3]) );
  and2 U15 ( .A(n14), .B(n7), .Y(rs2_sel[31]) );
  and2 U16 ( .A(n14), .B(n8), .Y(rs2_sel[30]) );
  and2 U17 ( .A(n13), .B(n8), .Y(rs2_sel[2]) );
  and2 U18 ( .A(n14), .B(n4), .Y(rs2_sel[29]) );
  and2 U19 ( .A(n14), .B(n5), .Y(rs2_sel[28]) );
  and2 U20 ( .A(n15), .B(n16), .Y(n14) );
  and2 U21 ( .A(n10), .B(n17), .Y(n15) );
  and2 U22 ( .A(n18), .B(n7), .Y(rs2_sel[27]) );
  and2 U23 ( .A(n18), .B(n8), .Y(rs2_sel[26]) );
  and2 U24 ( .A(n18), .B(n4), .Y(rs2_sel[25]) );
  and2 U25 ( .A(n18), .B(n5), .Y(rs2_sel[24]) );
  and2 U26 ( .A(n19), .B(n16), .Y(n18) );
  and2 U27 ( .A(n17), .B(n20), .Y(n19) );
  and2 U28 ( .A(n21), .B(n7), .Y(rs2_sel[23]) );
  and2 U29 ( .A(n21), .B(n8), .Y(rs2_sel[22]) );
  and2 U30 ( .A(n21), .B(n4), .Y(rs2_sel[21]) );
  and2 U31 ( .A(n21), .B(n5), .Y(rs2_sel[20]) );
  and2 U32 ( .A(n22), .B(n16), .Y(n21) );
  and2 U33 ( .A(n10), .B(n12), .Y(n22) );
  and2 U34 ( .A(n13), .B(n4), .Y(rs2_sel[1]) );
  and2 U35 ( .A(n23), .B(n7), .Y(rs2_sel[19]) );
  and2 U36 ( .A(n23), .B(n8), .Y(rs2_sel[18]) );
  and2 U37 ( .A(n23), .B(n4), .Y(rs2_sel[17]) );
  and2 U38 ( .A(n23), .B(n5), .Y(rs2_sel[16]) );
  and2 U39 ( .A(n24), .B(n16), .Y(n23) );
  inv U40 ( .A(n11), .Y(n16) );
  and2 U41 ( .A(n25), .B(n7), .Y(rs2_sel[15]) );
  and2 U42 ( .A(n25), .B(n8), .Y(rs2_sel[14]) );
  and2 U43 ( .A(n25), .B(n4), .Y(rs2_sel[13]) );
  nor2 U44 ( .A(n26), .B(n27), .Y(n4) );
  and2 U45 ( .A(n25), .B(n5), .Y(rs2_sel[12]) );
  and2 U46 ( .A(n28), .B(n10), .Y(n25) );
  inv U47 ( .A(n20), .Y(n10) );
  and2 U48 ( .A(n17), .B(n11), .Y(n28) );
  and2 U49 ( .A(n7), .B(n3), .Y(rs2_sel[11]) );
  nor2 U50 ( .A(n29), .B(n26), .Y(n7) );
  and2 U51 ( .A(n8), .B(n3), .Y(rs2_sel[10]) );
  and2 U52 ( .A(n30), .B(n17), .Y(n3) );
  inv U53 ( .A(n12), .Y(n17) );
  and2 U54 ( .A(n11), .B(n20), .Y(n30) );
  nor2 U55 ( .A(n29), .B(n31), .Y(n8) );
  and2 U56 ( .A(n13), .B(n5), .Y(rs2_sel[0]) );
  nor2 U57 ( .A(n27), .B(n31), .Y(n5) );
  inv U58 ( .A(n26), .Y(n31) );
  or2 U59 ( .A(n32), .B(n33), .Y(n26) );
  inv U60 ( .A(n29), .Y(n27) );
  or2 U61 ( .A(n34), .B(n33), .Y(n29) );
  and2 U62 ( .A(n24), .B(n11), .Y(n13) );
  or2 U63 ( .A(n35), .B(n33), .Y(n11) );
  and2 U64 ( .A(n12), .B(n20), .Y(n24) );
  or2 U65 ( .A(n36), .B(n33), .Y(n20) );
  or2 U66 ( .A(n37), .B(n33), .Y(n12) );
  and2 U67 ( .A(n38), .B(n39), .Y(rs1_sel[9]) );
  and2 U68 ( .A(n40), .B(n38), .Y(rs1_sel[8]) );
  and2 U69 ( .A(n41), .B(n42), .Y(rs1_sel[7]) );
  and2 U70 ( .A(n43), .B(n41), .Y(rs1_sel[6]) );
  and2 U71 ( .A(n41), .B(n39), .Y(rs1_sel[5]) );
  and2 U72 ( .A(n41), .B(n40), .Y(rs1_sel[4]) );
  and2 U73 ( .A(n44), .B(n45), .Y(n41) );
  and2 U74 ( .A(n46), .B(n47), .Y(n44) );
  and2 U75 ( .A(n48), .B(n42), .Y(rs1_sel[3]) );
  and2 U76 ( .A(n49), .B(n42), .Y(rs1_sel[31]) );
  and2 U77 ( .A(n49), .B(n43), .Y(rs1_sel[30]) );
  and2 U78 ( .A(n48), .B(n43), .Y(rs1_sel[2]) );
  and2 U79 ( .A(n49), .B(n39), .Y(rs1_sel[29]) );
  and2 U80 ( .A(n49), .B(n40), .Y(rs1_sel[28]) );
  and2 U81 ( .A(n50), .B(n51), .Y(n49) );
  and2 U82 ( .A(n45), .B(n52), .Y(n50) );
  and2 U83 ( .A(n53), .B(n42), .Y(rs1_sel[27]) );
  and2 U84 ( .A(n53), .B(n43), .Y(rs1_sel[26]) );
  and2 U85 ( .A(n53), .B(n39), .Y(rs1_sel[25]) );
  and2 U86 ( .A(n53), .B(n40), .Y(rs1_sel[24]) );
  and2 U87 ( .A(n54), .B(n51), .Y(n53) );
  and2 U88 ( .A(n52), .B(n55), .Y(n54) );
  and2 U89 ( .A(n56), .B(n42), .Y(rs1_sel[23]) );
  and2 U90 ( .A(n56), .B(n43), .Y(rs1_sel[22]) );
  and2 U91 ( .A(n56), .B(n39), .Y(rs1_sel[21]) );
  and2 U92 ( .A(n56), .B(n40), .Y(rs1_sel[20]) );
  and2 U93 ( .A(n57), .B(n51), .Y(n56) );
  and2 U94 ( .A(n45), .B(n47), .Y(n57) );
  and2 U95 ( .A(n48), .B(n39), .Y(rs1_sel[1]) );
  and2 U96 ( .A(n58), .B(n42), .Y(rs1_sel[19]) );
  and2 U97 ( .A(n58), .B(n43), .Y(rs1_sel[18]) );
  and2 U98 ( .A(n58), .B(n39), .Y(rs1_sel[17]) );
  and2 U99 ( .A(n58), .B(n40), .Y(rs1_sel[16]) );
  and2 U100 ( .A(n59), .B(n51), .Y(n58) );
  inv U101 ( .A(n46), .Y(n51) );
  and2 U102 ( .A(n60), .B(n42), .Y(rs1_sel[15]) );
  and2 U103 ( .A(n60), .B(n43), .Y(rs1_sel[14]) );
  and2 U104 ( .A(n60), .B(n39), .Y(rs1_sel[13]) );
  nor2 U105 ( .A(n61), .B(n62), .Y(n39) );
  and2 U106 ( .A(n60), .B(n40), .Y(rs1_sel[12]) );
  and2 U107 ( .A(n63), .B(n45), .Y(n60) );
  inv U108 ( .A(n55), .Y(n45) );
  and2 U109 ( .A(n52), .B(n46), .Y(n63) );
  and2 U110 ( .A(n42), .B(n38), .Y(rs1_sel[11]) );
  nor2 U111 ( .A(n64), .B(n61), .Y(n42) );
  and2 U112 ( .A(n43), .B(n38), .Y(rs1_sel[10]) );
  and2 U113 ( .A(n65), .B(n52), .Y(n38) );
  inv U114 ( .A(n47), .Y(n52) );
  and2 U115 ( .A(n46), .B(n55), .Y(n65) );
  nor2 U116 ( .A(n64), .B(n66), .Y(n43) );
  and2 U117 ( .A(n48), .B(n40), .Y(rs1_sel[0]) );
  nor2 U118 ( .A(n62), .B(n66), .Y(n40) );
  inv U119 ( .A(n61), .Y(n66) );
  nand2 U120 ( .A(imem_rdata[15]), .B(n67), .Y(n61) );
  inv U121 ( .A(n64), .Y(n62) );
  nand2 U122 ( .A(imem_rdata[16]), .B(n67), .Y(n64) );
  and2 U123 ( .A(n59), .B(n46), .Y(n48) );
  nand2 U124 ( .A(imem_rdata[19]), .B(n67), .Y(n46) );
  and2 U125 ( .A(n47), .B(n55), .Y(n59) );
  nand2 U126 ( .A(imem_rdata[17]), .B(n67), .Y(n55) );
  nand2 U127 ( .A(imem_rdata[18]), .B(n67), .Y(n47) );
  nand2 U128 ( .A(n68), .B(n33), .Y(n67) );
  nor2 U129 ( .A(n69), .B(n70), .Y(n33) );
  and2 U130 ( .A(n71), .B(n72), .Y(rd_sel[9]) );
  and2 U131 ( .A(n73), .B(n71), .Y(rd_sel[8]) );
  and2 U132 ( .A(n74), .B(n75), .Y(rd_sel[7]) );
  and2 U133 ( .A(n76), .B(n74), .Y(rd_sel[6]) );
  and2 U134 ( .A(n74), .B(n72), .Y(rd_sel[5]) );
  and2 U135 ( .A(n74), .B(n73), .Y(rd_sel[4]) );
  and2 U136 ( .A(n77), .B(n78), .Y(n74) );
  and2 U137 ( .A(n79), .B(n80), .Y(n77) );
  and2 U138 ( .A(n81), .B(n75), .Y(rd_sel[3]) );
  and2 U139 ( .A(n82), .B(n75), .Y(rd_sel[31]) );
  and2 U140 ( .A(n82), .B(n76), .Y(rd_sel[30]) );
  and2 U141 ( .A(n81), .B(n76), .Y(rd_sel[2]) );
  and2 U142 ( .A(n82), .B(n72), .Y(rd_sel[29]) );
  and2 U143 ( .A(n82), .B(n73), .Y(rd_sel[28]) );
  and2 U144 ( .A(n83), .B(n84), .Y(n82) );
  and2 U145 ( .A(n78), .B(n85), .Y(n83) );
  and2 U146 ( .A(n86), .B(n75), .Y(rd_sel[27]) );
  and2 U147 ( .A(n86), .B(n76), .Y(rd_sel[26]) );
  and2 U148 ( .A(n86), .B(n72), .Y(rd_sel[25]) );
  and2 U149 ( .A(n86), .B(n73), .Y(rd_sel[24]) );
  and2 U150 ( .A(n87), .B(n84), .Y(n86) );
  and2 U151 ( .A(n85), .B(n88), .Y(n87) );
  and2 U152 ( .A(n89), .B(n75), .Y(rd_sel[23]) );
  and2 U153 ( .A(n89), .B(n76), .Y(rd_sel[22]) );
  and2 U154 ( .A(n89), .B(n72), .Y(rd_sel[21]) );
  and2 U155 ( .A(n89), .B(n73), .Y(rd_sel[20]) );
  and2 U156 ( .A(n90), .B(n84), .Y(n89) );
  and2 U157 ( .A(n78), .B(n80), .Y(n90) );
  and2 U158 ( .A(n81), .B(n72), .Y(rd_sel[1]) );
  and2 U159 ( .A(n91), .B(n75), .Y(rd_sel[19]) );
  and2 U160 ( .A(n91), .B(n76), .Y(rd_sel[18]) );
  and2 U161 ( .A(n91), .B(n72), .Y(rd_sel[17]) );
  and2 U162 ( .A(n91), .B(n73), .Y(rd_sel[16]) );
  and2 U163 ( .A(n92), .B(n84), .Y(n91) );
  inv U164 ( .A(n79), .Y(n84) );
  and2 U165 ( .A(n93), .B(n75), .Y(rd_sel[15]) );
  and2 U166 ( .A(n93), .B(n76), .Y(rd_sel[14]) );
  and2 U167 ( .A(n93), .B(n72), .Y(rd_sel[13]) );
  and2 U168 ( .A(n94), .B(n95), .Y(n72) );
  nor2 U169 ( .A(n96), .B(clk), .Y(n94) );
  and2 U170 ( .A(n93), .B(n73), .Y(rd_sel[12]) );
  and2 U171 ( .A(n97), .B(n78), .Y(n93) );
  inv U172 ( .A(n88), .Y(n78) );
  and2 U173 ( .A(n85), .B(n79), .Y(n97) );
  and2 U174 ( .A(n75), .B(n71), .Y(rd_sel[11]) );
  and2 U175 ( .A(n98), .B(n96), .Y(n75) );
  and2 U176 ( .A(n95), .B(n1), .Y(n98) );
  and2 U177 ( .A(n76), .B(n71), .Y(rd_sel[10]) );
  and2 U178 ( .A(n99), .B(n85), .Y(n71) );
  inv U179 ( .A(n80), .Y(n85) );
  and2 U180 ( .A(n79), .B(n88), .Y(n99) );
  and2 U181 ( .A(n100), .B(n96), .Y(n76) );
  and2 U182 ( .A(n101), .B(n1), .Y(n100) );
  and2 U183 ( .A(n73), .B(n81), .Y(rd_sel[0]) );
  and2 U184 ( .A(n92), .B(n79), .Y(n81) );
  nand2 U185 ( .A(imem_rdata[11]), .B(n102), .Y(n79) );
  and2 U186 ( .A(n80), .B(n88), .Y(n92) );
  nand2 U187 ( .A(imem_rdata[9]), .B(n102), .Y(n88) );
  nand2 U188 ( .A(imem_rdata[10]), .B(n102), .Y(n80) );
  and2 U189 ( .A(n103), .B(n1), .Y(n73) );
  nor2 U190 ( .A(n96), .B(n95), .Y(n103) );
  inv U191 ( .A(n101), .Y(n95) );
  nand2 U192 ( .A(imem_rdata[7]), .B(n102), .Y(n101) );
  and2 U193 ( .A(imem_rdata[8]), .B(n102), .Y(n96) );
  nand2 U194 ( .A(n104), .B(n105), .Y(n102) );
  aoi21 U195 ( .A(n106), .B(n107), .C(n70), .Y(n104) );
  nand2 U196 ( .A(n108), .B(n109), .Y(rd_mux_sel[2]) );
  or2 U197 ( .A(n110), .B(n111), .Y(rd_mux_sel[1]) );
  inv U198 ( .A(n112), .Y(n110) );
  oai21 U199 ( .A(n113), .B(n114), .C(n115), .Y(rd_mux_sel[0]) );
  and2 U200 ( .A(n112), .B(n109), .Y(n115) );
  oai21 U201 ( .A(n116), .B(n117), .C(n108), .Y(pc_mux_sel) );
  nor2 U202 ( .A(n118), .B(n119), .Y(n108) );
  nor2 U203 ( .A(n120), .B(n121), .Y(imm[9]) );
  nor2 U204 ( .A(n122), .B(n121), .Y(imm[8]) );
  nor2 U205 ( .A(n123), .B(n121), .Y(imm[7]) );
  nor2 U206 ( .A(n124), .B(n121), .Y(imm[6]) );
  nor2 U207 ( .A(n125), .B(n121), .Y(imm[5]) );
  oai21 U208 ( .A(n105), .B(n35), .C(n126), .Y(imm[4]) );
  nand2 U209 ( .A(imem_rdata[11]), .B(n69), .Y(n126) );
  oai21 U210 ( .A(n105), .B(n37), .C(n127), .Y(imm[3]) );
  nand2 U211 ( .A(imem_rdata[10]), .B(n69), .Y(n127) );
  oai21 U212 ( .A(n128), .B(n129), .C(n130), .Y(imm[30]) );
  oai21 U213 ( .A(n105), .B(n36), .C(n131), .Y(imm[2]) );
  nand2 U214 ( .A(imem_rdata[9]), .B(n69), .Y(n131) );
  oai21 U215 ( .A(n128), .B(n120), .C(n130), .Y(imm[29]) );
  inv U216 ( .A(imem_rdata[29]), .Y(n120) );
  oai21 U217 ( .A(n128), .B(n122), .C(n130), .Y(imm[28]) );
  inv U218 ( .A(imem_rdata[28]), .Y(n122) );
  oai21 U219 ( .A(n128), .B(n123), .C(n130), .Y(imm[27]) );
  inv U220 ( .A(imem_rdata[27]), .Y(n123) );
  oai21 U221 ( .A(n128), .B(n124), .C(n130), .Y(imm[26]) );
  inv U222 ( .A(imem_rdata[26]), .Y(n124) );
  oai21 U223 ( .A(n128), .B(n125), .C(n130), .Y(imm[25]) );
  inv U224 ( .A(imem_rdata[25]), .Y(n125) );
  oai21 U225 ( .A(n128), .B(n35), .C(n130), .Y(imm[24]) );
  inv U226 ( .A(imem_rdata[24]), .Y(n35) );
  oai21 U227 ( .A(n128), .B(n37), .C(n130), .Y(imm[23]) );
  inv U228 ( .A(imem_rdata[23]), .Y(n37) );
  oai21 U229 ( .A(n128), .B(n36), .C(n130), .Y(imm[22]) );
  inv U230 ( .A(imem_rdata[22]), .Y(n36) );
  oai21 U231 ( .A(n128), .B(n34), .C(n130), .Y(imm[21]) );
  oai21 U232 ( .A(n128), .B(n32), .C(n130), .Y(imm[20]) );
  or2 U233 ( .A(n132), .B(n121), .Y(n130) );
  inv U234 ( .A(imem_rdata[20]), .Y(n32) );
  oai21 U235 ( .A(n105), .B(n34), .C(n133), .Y(imm[1]) );
  nand2 U236 ( .A(imem_rdata[8]), .B(n69), .Y(n133) );
  inv U237 ( .A(imem_rdata[21]), .Y(n34) );
  oai21 U238 ( .A(n134), .B(n135), .C(n136), .Y(imm[19]) );
  inv U239 ( .A(imem_rdata[19]), .Y(n135) );
  oai21 U240 ( .A(n134), .B(n137), .C(n136), .Y(imm[18]) );
  inv U241 ( .A(imem_rdata[18]), .Y(n137) );
  oai21 U242 ( .A(n134), .B(n138), .C(n136), .Y(imm[17]) );
  inv U243 ( .A(imem_rdata[17]), .Y(n138) );
  oai21 U244 ( .A(n134), .B(n139), .C(n136), .Y(imm[16]) );
  inv U245 ( .A(imem_rdata[16]), .Y(n139) );
  oai21 U246 ( .A(n134), .B(n140), .C(n136), .Y(imm[15]) );
  inv U247 ( .A(imem_rdata[15]), .Y(n140) );
  oai21 U248 ( .A(n134), .B(n141), .C(n136), .Y(imm[14]) );
  oai21 U249 ( .A(n134), .B(n142), .C(n136), .Y(imm[13]) );
  oai21 U250 ( .A(n134), .B(n143), .C(n136), .Y(imm[12]) );
  oai21 U251 ( .A(n69), .B(n144), .C(imem_rdata[31]), .Y(n136) );
  and2 U252 ( .A(n128), .B(n145), .Y(n134) );
  and2 U253 ( .A(n112), .B(n146), .Y(n128) );
  nand2 U254 ( .A(n147), .B(n106), .Y(n112) );
  and2 U255 ( .A(n107), .B(imem_rdata[5]), .Y(n147) );
  oai21 U256 ( .A(n148), .B(n117), .C(n149), .Y(imm[11]) );
  aoi21 U257 ( .A(n119), .B(imem_rdata[20]), .C(n150), .Y(n149) );
  aoi21 U258 ( .A(n68), .B(n151), .C(n132), .Y(n150) );
  inv U259 ( .A(imem_rdata[31]), .Y(n132) );
  inv U260 ( .A(n144), .Y(n68) );
  nor2 U261 ( .A(n129), .B(n121), .Y(imm[10]) );
  inv U262 ( .A(imem_rdata[30]), .Y(n129) );
  oai21 U263 ( .A(n148), .B(n151), .C(n152), .Y(imm[0]) );
  nand2 U264 ( .A(imem_rdata[20]), .B(n144), .Y(n152) );
  inv U265 ( .A(imem_rdata[7]), .Y(n148) );
  inv U266 ( .A(n153), .Y(dmem_wmask[1]) );
  aoi21 U267 ( .A(n154), .B(n155), .C(dmem_wmask[3]), .Y(n153) );
  and2 U268 ( .A(n155), .B(n156), .Y(dmem_wmask[3]) );
  and2 U269 ( .A(n155), .B(n157), .Y(dmem_wmask[0]) );
  and2 U270 ( .A(dmem_write), .B(n141), .Y(n155) );
  inv U271 ( .A(n151), .Y(dmem_write) );
  oai21 U272 ( .A(n116), .B(n117), .C(n158), .Y(cmp_out) );
  nand2 U273 ( .A(n159), .B(n111), .Y(n158) );
  and2 U274 ( .A(n160), .B(N204), .Y(n111) );
  nor2 U275 ( .A(n113), .B(N205), .Y(n160) );
  mux2 U276 ( .A(n161), .B(cmp_lt), .S0(N203), .Y(n159) );
  and2 U277 ( .A(n162), .B(n163), .Y(n116) );
  mux2 U278 ( .A(n164), .B(n165), .S0(N203), .Y(n163) );
  or2 U279 ( .A(N205), .B(cmp_eq), .Y(n165) );
  aoi21 U280 ( .A(cmp_eq), .B(n141), .C(n166), .Y(n164) );
  mux2 U281 ( .A(n167), .B(cmp_lt), .S0(N204), .Y(n166) );
  and2 U282 ( .A(N205), .B(n161), .Y(n167) );
  inv U283 ( .A(n168), .Y(n161) );
  aoi21 U284 ( .A(n168), .B(shift_dir), .C(n169), .Y(n162) );
  nor2 U285 ( .A(n157), .B(cmp_lt), .Y(n169) );
  or2 U286 ( .A(n142), .B(n143), .Y(n157) );
  nor2 U287 ( .A(n141), .B(n114), .Y(shift_dir) );
  inv U288 ( .A(n154), .Y(n114) );
  xor2 U289 ( .A(n170), .B(cmp_a_31), .Y(n168) );
  xnor2 U290 ( .A(cmp_lt), .B(cmp_b_31), .Y(n170) );
  and2 U291 ( .A(n171), .B(N204), .Y(alu_op[1]) );
  nor2 U292 ( .A(n141), .B(n113), .Y(n171) );
  and2 U293 ( .A(n172), .B(n173), .Y(alu_op[0]) );
  nor2 U294 ( .A(n154), .B(n113), .Y(n173) );
  nor2 U295 ( .A(n70), .B(cmp_mux_sel), .Y(n113) );
  and2 U296 ( .A(N203), .B(n142), .Y(n154) );
  nor2 U297 ( .A(n141), .B(n156), .Y(n172) );
  and2 U298 ( .A(N204), .B(n143), .Y(n156) );
  nand2 U299 ( .A(n121), .B(n146), .Y(alu_mux_2_sel) );
  and2 U300 ( .A(n105), .B(n174), .Y(n121) );
  inv U301 ( .A(n69), .Y(n174) );
  nand2 U302 ( .A(n117), .B(n151), .Y(n69) );
  nand2 U303 ( .A(n175), .B(n176), .Y(n151) );
  nor2 U304 ( .A(n177), .B(imem_rdata[4]), .Y(n175) );
  inv U305 ( .A(n178), .Y(n177) );
  nor2 U306 ( .A(n144), .B(n119), .Y(n105) );
  inv U307 ( .A(n145), .Y(n119) );
  nand2 U308 ( .A(n179), .B(n109), .Y(n144) );
  nand2 U309 ( .A(n180), .B(n181), .Y(n109) );
  nor2 U310 ( .A(imem_rdata[4]), .B(imem_rdata[5]), .Y(n181) );
  and2 U311 ( .A(n182), .B(n178), .Y(n180) );
  nor2 U312 ( .A(n118), .B(cmp_mux_sel), .Y(n179) );
  and2 U313 ( .A(n183), .B(n107), .Y(cmp_mux_sel) );
  and2 U314 ( .A(n182), .B(n184), .Y(n183) );
  nor2 U315 ( .A(n185), .B(imem_rdata[3]), .Y(n118) );
  inv U316 ( .A(n186), .Y(n185) );
  nand2 U317 ( .A(n187), .B(n146), .Y(alu_mux_1_sel) );
  nand2 U318 ( .A(n188), .B(n106), .Y(n146) );
  and2 U319 ( .A(n107), .B(n184), .Y(n188) );
  and2 U320 ( .A(n145), .B(n117), .Y(n187) );
  nand2 U321 ( .A(n189), .B(n190), .Y(n117) );
  nor2 U322 ( .A(imem_rdata[3]), .B(imem_rdata[4]), .Y(n190) );
  and2 U323 ( .A(imem_rdata[6]), .B(n176), .Y(n189) );
  nand2 U324 ( .A(imem_rdata[3]), .B(n186), .Y(n145) );
  and2 U325 ( .A(n191), .B(n192), .Y(n186) );
  nor2 U326 ( .A(n184), .B(imem_rdata[4]), .Y(n192) );
  inv U327 ( .A(imem_rdata[5]), .Y(n184) );
  and2 U328 ( .A(n106), .B(imem_rdata[6]), .Y(n191) );
  and2 U329 ( .A(imem_rdata[2]), .B(n193), .Y(n106) );
  and2 U330 ( .A(n194), .B(n195), .Y(alu_inv_rs2) );
  and2 U331 ( .A(n196), .B(n143), .Y(n195) );
  inv U332 ( .A(N203), .Y(n143) );
  and2 U333 ( .A(n142), .B(n141), .Y(n196) );
  inv U334 ( .A(N205), .Y(n141) );
  inv U335 ( .A(N204), .Y(n142) );
  and2 U336 ( .A(n70), .B(imem_rdata[30]), .Y(n194) );
  and2 U337 ( .A(n107), .B(n176), .Y(n70) );
  and2 U338 ( .A(imem_rdata[5]), .B(n182), .Y(n176) );
  nor2 U339 ( .A(n197), .B(imem_rdata[2]), .Y(n182) );
  inv U340 ( .A(n193), .Y(n197) );
  and2 U341 ( .A(imem_rdata[1]), .B(imem_rdata[0]), .Y(n193) );
  and2 U342 ( .A(imem_rdata[4]), .B(n178), .Y(n107) );
  nor2 U343 ( .A(imem_rdata[6]), .B(imem_rdata[3]), .Y(n178) );
endmodule


module mem_mux_0 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_0 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_0 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_0 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n153, n154, n155, n156, n157, n158, n159, n160, n161, n162, n163,
         n164, n165, n166, n167, n168, n169, n170, n171, n172, n173, n174,
         n175, n176, n177, n178, n179, n180, n181, n182, n183, n1, n2, n3, n4,
         n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16, n17, n18, n19,
         n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30, n31, n32, n33,
         n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44, n45, n46, n47,
         n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58, n59, n60, n61,
         n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72, n73, n74, n75,
         n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86, n87, n88, n89,
         n90, n91, n92;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n183), .CLK(n1), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n182), .CLK(n1), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n181), .CLK(n1), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n180), .CLK(n1), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n179), .CLK(n1), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n178), .CLK(n1), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n177), .CLK(n1), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n176), .CLK(n1), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n175), .CLK(n1), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n174), .CLK(n1), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n173), .CLK(n1), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n172), .CLK(n1), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n171), .CLK(n2), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n170), .CLK(n2), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n169), .CLK(n2), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n168), .CLK(n2), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n167), .CLK(n2), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n166), .CLK(n2), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n165), .CLK(n2), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n164), .CLK(n2), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n163), .CLK(n2), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n162), .CLK(n2), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n161), .CLK(n2), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n160), .CLK(n2), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n159), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n158), .CLK(n2), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n157), .CLK(n1), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n156), .CLK(n2), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n155), .CLK(n1), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n154), .CLK(n2), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n153), .CLK(n1), .Q(rf_data[1]) );
  \buf  U3 ( .A(clk), .Y(n1) );
  \buf  U4 ( .A(clk), .Y(n2) );
  nand2 U5 ( .A(n3), .B(n4), .Y(rs2_rdata) );
  and2 U6 ( .A(n5), .B(n6), .Y(n4) );
  and2 U7 ( .A(n7), .B(n8), .Y(n6) );
  and2 U8 ( .A(n9), .B(n10), .Y(n8) );
  aoi21 U9 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n11), .Y(n10) );
  and2 U10 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n11) );
  aoi21 U11 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n12), .Y(n9) );
  and2 U12 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n12) );
  and2 U13 ( .A(n13), .B(n14), .Y(n7) );
  aoi21 U14 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n15), .Y(n14) );
  and2 U15 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n15) );
  aoi21 U16 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n16), .Y(n13) );
  and2 U17 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n16) );
  and2 U18 ( .A(n17), .B(n18), .Y(n5) );
  and2 U19 ( .A(n19), .B(n20), .Y(n18) );
  aoi21 U20 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n21), .Y(n20) );
  and2 U21 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n21) );
  aoi21 U22 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n22), .Y(n19) );
  and2 U23 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n22) );
  and2 U24 ( .A(n23), .B(n24), .Y(n17) );
  aoi21 U25 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n25), .Y(n24) );
  and2 U26 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n25) );
  aoi21 U27 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n26), .Y(n23) );
  and2 U28 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n26) );
  and2 U29 ( .A(n27), .B(n28), .Y(n3) );
  and2 U30 ( .A(n29), .B(n30), .Y(n28) );
  and2 U31 ( .A(n31), .B(n32), .Y(n30) );
  aoi21 U32 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n33), .Y(n32) );
  and2 U33 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n33) );
  aoi21 U34 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n34), .Y(n31) );
  and2 U35 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n34) );
  and2 U36 ( .A(n35), .B(n36), .Y(n29) );
  aoi21 U37 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n37), .Y(n36) );
  and2 U38 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n37) );
  aoi21 U39 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n38), .Y(n35) );
  and2 U40 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n38) );
  and2 U41 ( .A(n39), .B(n40), .Y(n27) );
  and2 U42 ( .A(n41), .B(n42), .Y(n40) );
  aoi21 U43 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n43), .Y(n42) );
  and2 U44 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n43) );
  aoi21 U45 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n44), .Y(n41) );
  and2 U46 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n44) );
  and2 U47 ( .A(n45), .B(n46), .Y(n39) );
  nand2 U48 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n46) );
  aoi21 U49 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n47), .Y(n45) );
  and2 U50 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n47) );
  nand2 U51 ( .A(n48), .B(n49), .Y(rs1_rdata) );
  and2 U52 ( .A(n50), .B(n51), .Y(n49) );
  and2 U53 ( .A(n52), .B(n53), .Y(n51) );
  and2 U54 ( .A(n54), .B(n55), .Y(n53) );
  aoi21 U55 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n56), .Y(n55) );
  and2 U56 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n56) );
  aoi21 U57 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n57), .Y(n54) );
  and2 U58 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n57) );
  and2 U59 ( .A(n58), .B(n59), .Y(n52) );
  aoi21 U60 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n60), .Y(n59) );
  and2 U61 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n60) );
  aoi21 U62 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n61), .Y(n58) );
  and2 U63 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n61) );
  and2 U64 ( .A(n62), .B(n63), .Y(n50) );
  and2 U65 ( .A(n64), .B(n65), .Y(n63) );
  aoi21 U66 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n66), .Y(n65) );
  and2 U67 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n66) );
  aoi21 U68 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n67), .Y(n64) );
  and2 U69 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n67) );
  and2 U70 ( .A(n68), .B(n69), .Y(n62) );
  aoi21 U71 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n70), .Y(n69) );
  and2 U72 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n70) );
  aoi21 U73 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n71), .Y(n68) );
  and2 U74 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n71) );
  and2 U75 ( .A(n72), .B(n73), .Y(n48) );
  and2 U76 ( .A(n74), .B(n75), .Y(n73) );
  and2 U77 ( .A(n76), .B(n77), .Y(n75) );
  aoi21 U78 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n78), .Y(n77) );
  and2 U79 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n78) );
  aoi21 U80 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n79), .Y(n76) );
  and2 U81 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n79) );
  and2 U82 ( .A(n80), .B(n81), .Y(n74) );
  aoi21 U83 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n82), .Y(n81) );
  and2 U84 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n82) );
  aoi21 U85 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n83), .Y(n80) );
  and2 U86 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n83) );
  and2 U87 ( .A(n84), .B(n85), .Y(n72) );
  and2 U88 ( .A(n86), .B(n87), .Y(n85) );
  aoi21 U89 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n88), .Y(n87) );
  and2 U90 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n88) );
  aoi21 U91 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n89), .Y(n86) );
  and2 U92 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n89) );
  and2 U93 ( .A(n90), .B(n91), .Y(n84) );
  nand2 U94 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n91) );
  aoi21 U95 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n92), .Y(n90) );
  and2 U96 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n92) );
  mux2 U97 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n183) );
  mux2 U98 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n182) );
  mux2 U99 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n181) );
  mux2 U100 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n180) );
  mux2 U101 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n179) );
  mux2 U102 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n178) );
  mux2 U103 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n177) );
  mux2 U104 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n176) );
  mux2 U105 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n175) );
  mux2 U106 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n174) );
  mux2 U107 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n173) );
  mux2 U108 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n172) );
  mux2 U109 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n171) );
  mux2 U110 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n170) );
  mux2 U111 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n169) );
  mux2 U112 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n168) );
  mux2 U113 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n167) );
  mux2 U114 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n166) );
  mux2 U115 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n165) );
  mux2 U116 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n164) );
  mux2 U117 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n163) );
  mux2 U118 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n162) );
  mux2 U119 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n161) );
  mux2 U120 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n160) );
  mux2 U121 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n159) );
  mux2 U122 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n158) );
  mux2 U123 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n157) );
  mux2 U124 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n156) );
  mux2 U125 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n155) );
  mux2 U126 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n154) );
  mux2 U127 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n153) );
endmodule


module pc_0 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_0 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_0 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_0 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_0 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__0 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__95 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__94 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_0 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_0 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_0 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_0 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_0 regfile ( .clk(clk), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_0 pc_ ( .clk(clk), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_0 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_0 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_0 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_0 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__0 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__95 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__94 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
endmodule


module mem_mux_31 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_31 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_31 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_31 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_31 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_31 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_31 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_31 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_31 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__93 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__92 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__91 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_31 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_31 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_31 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_31 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_31 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_31 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_31 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_31 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_31 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_31 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__93 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__92 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__91 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_30 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_30 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_30 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_30 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_30 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_30 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_30 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_30 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_30 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__90 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__89 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__88 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_30 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_30 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_30 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_30 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_30 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_30 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_30 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_30 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_30 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_30 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__90 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__89 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__88 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_29 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_29 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_29 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_29 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_29 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_29 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_29 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_29 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_29 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__87 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__86 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__85 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_29 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_29 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_29 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_29 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_29 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_29 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_29 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_29 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_29 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_29 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__87 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__86 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__85 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_28 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_28 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_28 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_28 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_28 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_28 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_28 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_28 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_28 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__84 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__83 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__82 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_28 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_28 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_28 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_28 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_28 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_28 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_28 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_28 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_28 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_28 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__84 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__83 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__82 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_27 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_27 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_27 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_27 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_27 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_27 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_27 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_27 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_27 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__81 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__80 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__79 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_27 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_27 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_27 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_27 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_27 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_27 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_27 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_27 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_27 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_27 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__81 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__80 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__79 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module mem_mux_26 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_26 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_26 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_26 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_26 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_26 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_26 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_26 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_26 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__78 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__77 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__76 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_26 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_26 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_26 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_26 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_26 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_26 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_26 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_26 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_26 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_26 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__78 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__77 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__76 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module mem_mux_25 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_25 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_25 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_25 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_25 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_25 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_25 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_25 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_25 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__75 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__74 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__73 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_25 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_25 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_25 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_25 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_25 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_25 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_25 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_25 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_25 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_25 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__75 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__74 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__73 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module mem_mux_24 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_24 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_24 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_24 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121, n122,
         n123;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n93), .CLK(n1), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n94), .CLK(n1), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n95), .CLK(n1), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n96), .CLK(n1), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n97), .CLK(n1), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n98), .CLK(n1), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n99), .CLK(n1), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n100), .CLK(n1), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n101), .CLK(n1), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n102), .CLK(n1), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n103), .CLK(n1), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n104), .CLK(n1), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n105), .CLK(n2), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n106), .CLK(n2), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n107), .CLK(n2), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n108), .CLK(n2), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n109), .CLK(n2), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n110), .CLK(n2), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n111), .CLK(n2), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n112), .CLK(n2), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n113), .CLK(n2), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n114), .CLK(n2), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n115), .CLK(n2), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n116), .CLK(n2), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n117), .CLK(n2), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n118), .CLK(n1), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n119), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n120), .CLK(n2), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n121), .CLK(n1), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n122), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n123), .CLK(clk), .Q(rf_data[1]) );
  \buf  U3 ( .A(clk), .Y(n1) );
  \buf  U4 ( .A(clk), .Y(n2) );
  nand2 U5 ( .A(n3), .B(n4), .Y(rs2_rdata) );
  and2 U6 ( .A(n5), .B(n6), .Y(n4) );
  and2 U7 ( .A(n7), .B(n8), .Y(n6) );
  and2 U8 ( .A(n9), .B(n10), .Y(n8) );
  aoi21 U9 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n11), .Y(n10) );
  and2 U10 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n11) );
  aoi21 U11 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n12), .Y(n9) );
  and2 U12 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n12) );
  and2 U13 ( .A(n13), .B(n14), .Y(n7) );
  aoi21 U14 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n15), .Y(n14) );
  and2 U15 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n15) );
  aoi21 U16 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n16), .Y(n13) );
  and2 U17 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n16) );
  and2 U18 ( .A(n17), .B(n18), .Y(n5) );
  and2 U19 ( .A(n19), .B(n20), .Y(n18) );
  aoi21 U20 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n21), .Y(n20) );
  and2 U21 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n21) );
  aoi21 U22 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n22), .Y(n19) );
  and2 U23 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n22) );
  and2 U24 ( .A(n23), .B(n24), .Y(n17) );
  aoi21 U25 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n25), .Y(n24) );
  and2 U26 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n25) );
  aoi21 U27 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n26), .Y(n23) );
  and2 U28 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n26) );
  and2 U29 ( .A(n27), .B(n28), .Y(n3) );
  and2 U30 ( .A(n29), .B(n30), .Y(n28) );
  and2 U31 ( .A(n31), .B(n32), .Y(n30) );
  aoi21 U32 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n33), .Y(n32) );
  and2 U33 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n33) );
  aoi21 U34 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n34), .Y(n31) );
  and2 U35 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n34) );
  and2 U36 ( .A(n35), .B(n36), .Y(n29) );
  aoi21 U37 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n37), .Y(n36) );
  and2 U38 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n37) );
  aoi21 U39 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n38), .Y(n35) );
  and2 U40 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n38) );
  and2 U41 ( .A(n39), .B(n40), .Y(n27) );
  and2 U42 ( .A(n41), .B(n42), .Y(n40) );
  aoi21 U43 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n43), .Y(n42) );
  and2 U44 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n43) );
  aoi21 U45 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n44), .Y(n41) );
  and2 U46 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n44) );
  and2 U47 ( .A(n45), .B(n46), .Y(n39) );
  nand2 U48 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n46) );
  aoi21 U49 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n47), .Y(n45) );
  and2 U50 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n47) );
  nand2 U51 ( .A(n48), .B(n49), .Y(rs1_rdata) );
  and2 U52 ( .A(n50), .B(n51), .Y(n49) );
  and2 U53 ( .A(n52), .B(n53), .Y(n51) );
  and2 U54 ( .A(n54), .B(n55), .Y(n53) );
  aoi21 U55 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n56), .Y(n55) );
  and2 U56 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n56) );
  aoi21 U57 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n57), .Y(n54) );
  and2 U58 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n57) );
  and2 U59 ( .A(n58), .B(n59), .Y(n52) );
  aoi21 U60 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n60), .Y(n59) );
  and2 U61 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n60) );
  aoi21 U62 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n61), .Y(n58) );
  and2 U63 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n61) );
  and2 U64 ( .A(n62), .B(n63), .Y(n50) );
  and2 U65 ( .A(n64), .B(n65), .Y(n63) );
  aoi21 U66 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n66), .Y(n65) );
  and2 U67 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n66) );
  aoi21 U68 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n67), .Y(n64) );
  and2 U69 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n67) );
  and2 U70 ( .A(n68), .B(n69), .Y(n62) );
  aoi21 U71 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n70), .Y(n69) );
  and2 U72 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n70) );
  aoi21 U73 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n71), .Y(n68) );
  and2 U74 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n71) );
  and2 U75 ( .A(n72), .B(n73), .Y(n48) );
  and2 U76 ( .A(n74), .B(n75), .Y(n73) );
  and2 U77 ( .A(n76), .B(n77), .Y(n75) );
  aoi21 U78 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n78), .Y(n77) );
  and2 U79 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n78) );
  aoi21 U80 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n79), .Y(n76) );
  and2 U81 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n79) );
  and2 U82 ( .A(n80), .B(n81), .Y(n74) );
  aoi21 U83 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n82), .Y(n81) );
  and2 U84 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n82) );
  aoi21 U85 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n83), .Y(n80) );
  and2 U86 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n83) );
  and2 U87 ( .A(n84), .B(n85), .Y(n72) );
  and2 U88 ( .A(n86), .B(n87), .Y(n85) );
  aoi21 U89 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n88), .Y(n87) );
  and2 U90 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n88) );
  aoi21 U91 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n89), .Y(n86) );
  and2 U92 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n89) );
  and2 U93 ( .A(n90), .B(n91), .Y(n84) );
  nand2 U94 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n91) );
  aoi21 U95 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n92), .Y(n90) );
  and2 U96 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n92) );
  mux2 U97 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n93) );
  mux2 U98 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n94) );
  mux2 U99 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n95) );
  mux2 U100 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n96) );
  mux2 U101 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n97) );
  mux2 U102 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n98) );
  mux2 U103 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n99) );
  mux2 U104 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n100) );
  mux2 U105 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n101) );
  mux2 U106 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n102) );
  mux2 U107 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n103) );
  mux2 U108 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n104) );
  mux2 U109 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n105) );
  mux2 U110 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n106) );
  mux2 U111 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n107) );
  mux2 U112 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n108) );
  mux2 U113 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n109) );
  mux2 U114 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n110) );
  mux2 U115 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n111) );
  mux2 U116 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n112) );
  mux2 U117 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n113) );
  mux2 U118 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n114) );
  mux2 U119 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n115) );
  mux2 U120 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n116) );
  mux2 U121 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n117) );
  mux2 U122 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n118) );
  mux2 U123 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n119) );
  mux2 U124 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n120) );
  mux2 U125 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n121) );
  mux2 U126 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n122) );
  mux2 U127 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n123) );
endmodule


module pc_24 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_24 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_24 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_24 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_24 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__72 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__71 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__70 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_24 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_24 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_24 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_24 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_24 regfile ( .clk(clk), .rd_sel(rd_sel), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), 
        .rs2_rdata(rs2_rdata), .rf_data({rf_data[31:1], 
        SYNOPSYS_UNCONNECTED__0}) );
  pc_24 pc_ ( .clk(clk), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_24 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_24 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_24 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_24 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__72 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__71 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__70 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
endmodule


module mem_mux_23 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_23 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_23 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_23 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_23 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_23 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_23 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_23 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_23 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__69 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__68 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__67 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_23 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_23 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_23 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_23 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_23 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_23 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_23 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_23 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_23 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_23 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__69 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__68 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__67 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_22 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_22 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_22 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_22 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_22 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_22 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_22 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_22 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_22 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__66 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__65 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__64 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_22 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_22 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_22 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_22 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_22 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_22 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_22 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_22 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_22 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_22 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__66 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__65 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__64 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_21 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_21 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_21 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_21 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_21 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_21 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_21 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_21 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_21 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__63 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__62 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__61 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_21 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_21 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_21 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_21 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_21 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_21 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_21 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_21 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_21 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_21 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__63 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__62 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__61 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_20 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_20 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_20 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_20 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_20 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_20 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_20 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_20 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_20 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__60 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__59 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__58 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_20 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_20 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_20 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_20 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_20 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_20 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_20 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_20 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_20 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_20 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__60 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__59 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__58 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_19 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_19 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_19 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_19 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_19 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_19 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_19 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_19 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_19 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__57 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__56 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__55 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_19 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_19 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_19 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_19 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_19 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_19 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_19 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_19 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_19 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_19 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__57 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__56 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__55 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_18 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_18 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_18 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_18 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_18 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_18 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_18 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_18 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_18 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__54 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__53 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__52 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_18 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_18 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_18 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_18 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_18 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_18 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_18 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_18 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_18 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_18 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__54 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__53 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__52 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_17 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_17 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_17 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_17 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_17 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_17 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_17 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_17 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_17 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__51 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__50 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__49 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_17 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_17 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_17 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_17 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_17 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_17 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_17 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_17 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_17 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_17 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__51 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__50 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__49 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_16 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_16 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_16 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_16 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_16 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_16 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_16 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_16 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_16 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__48 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__47 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__46 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_16 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_16 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_16 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_16 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_16 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_16 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_16 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_16 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_16 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_16 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__48 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__47 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__46 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_15 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_15 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_15 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_15 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_15 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_15 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_15 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_15 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_15 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__45 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__44 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__43 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_15 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_15 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_15 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_15 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_15 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_15 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_15 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_15 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_15 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_15 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__45 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__44 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__43 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module mem_mux_14 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_14 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_14 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_14 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_14 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_14 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_14 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_14 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_14 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__42 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__41 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__40 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_14 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_14 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_14 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_14 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_14 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_14 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_14 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_14 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_14 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_14 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__42 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__41 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__40 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module mem_mux_13 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_13 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_13 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_13 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_13 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_13 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_13 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_13 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_13 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__39 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__38 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__37 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_13 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_13 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_13 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_13 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_13 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_13 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_13 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_13 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_13 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_13 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__39 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__38 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__37 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module mem_mux_12 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_12 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_12 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_12 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121, n122,
         n123;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n93), .CLK(n1), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n94), .CLK(n1), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n95), .CLK(n1), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n96), .CLK(n1), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n97), .CLK(n1), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n98), .CLK(n1), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n99), .CLK(n1), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n100), .CLK(n1), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n101), .CLK(n1), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n102), .CLK(n1), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n103), .CLK(n1), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n104), .CLK(n1), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n105), .CLK(n2), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n106), .CLK(n2), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n107), .CLK(n2), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n108), .CLK(n2), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n109), .CLK(n2), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n110), .CLK(n2), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n111), .CLK(n2), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n112), .CLK(n2), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n113), .CLK(n2), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n114), .CLK(n2), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n115), .CLK(n2), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n116), .CLK(n2), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n117), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n118), .CLK(n2), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n119), .CLK(n1), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n120), .CLK(n2), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n121), .CLK(n1), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n122), .CLK(n2), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n123), .CLK(n1), .Q(rf_data[1]) );
  \buf  U3 ( .A(clk), .Y(n1) );
  \buf  U4 ( .A(clk), .Y(n2) );
  nand2 U5 ( .A(n3), .B(n4), .Y(rs2_rdata) );
  and2 U6 ( .A(n5), .B(n6), .Y(n4) );
  and2 U7 ( .A(n7), .B(n8), .Y(n6) );
  and2 U8 ( .A(n9), .B(n10), .Y(n8) );
  aoi21 U9 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n11), .Y(n10) );
  and2 U10 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n11) );
  aoi21 U11 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n12), .Y(n9) );
  and2 U12 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n12) );
  and2 U13 ( .A(n13), .B(n14), .Y(n7) );
  aoi21 U14 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n15), .Y(n14) );
  and2 U15 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n15) );
  aoi21 U16 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n16), .Y(n13) );
  and2 U17 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n16) );
  and2 U18 ( .A(n17), .B(n18), .Y(n5) );
  and2 U19 ( .A(n19), .B(n20), .Y(n18) );
  aoi21 U20 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n21), .Y(n20) );
  and2 U21 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n21) );
  aoi21 U22 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n22), .Y(n19) );
  and2 U23 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n22) );
  and2 U24 ( .A(n23), .B(n24), .Y(n17) );
  aoi21 U25 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n25), .Y(n24) );
  and2 U26 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n25) );
  aoi21 U27 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n26), .Y(n23) );
  and2 U28 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n26) );
  and2 U29 ( .A(n27), .B(n28), .Y(n3) );
  and2 U30 ( .A(n29), .B(n30), .Y(n28) );
  and2 U31 ( .A(n31), .B(n32), .Y(n30) );
  aoi21 U32 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n33), .Y(n32) );
  and2 U33 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n33) );
  aoi21 U34 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n34), .Y(n31) );
  and2 U35 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n34) );
  and2 U36 ( .A(n35), .B(n36), .Y(n29) );
  aoi21 U37 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n37), .Y(n36) );
  and2 U38 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n37) );
  aoi21 U39 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n38), .Y(n35) );
  and2 U40 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n38) );
  and2 U41 ( .A(n39), .B(n40), .Y(n27) );
  and2 U42 ( .A(n41), .B(n42), .Y(n40) );
  aoi21 U43 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n43), .Y(n42) );
  and2 U44 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n43) );
  aoi21 U45 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n44), .Y(n41) );
  and2 U46 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n44) );
  and2 U47 ( .A(n45), .B(n46), .Y(n39) );
  nand2 U48 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n46) );
  aoi21 U49 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n47), .Y(n45) );
  and2 U50 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n47) );
  nand2 U51 ( .A(n48), .B(n49), .Y(rs1_rdata) );
  and2 U52 ( .A(n50), .B(n51), .Y(n49) );
  and2 U53 ( .A(n52), .B(n53), .Y(n51) );
  and2 U54 ( .A(n54), .B(n55), .Y(n53) );
  aoi21 U55 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n56), .Y(n55) );
  and2 U56 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n56) );
  aoi21 U57 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n57), .Y(n54) );
  and2 U58 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n57) );
  and2 U59 ( .A(n58), .B(n59), .Y(n52) );
  aoi21 U60 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n60), .Y(n59) );
  and2 U61 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n60) );
  aoi21 U62 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n61), .Y(n58) );
  and2 U63 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n61) );
  and2 U64 ( .A(n62), .B(n63), .Y(n50) );
  and2 U65 ( .A(n64), .B(n65), .Y(n63) );
  aoi21 U66 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n66), .Y(n65) );
  and2 U67 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n66) );
  aoi21 U68 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n67), .Y(n64) );
  and2 U69 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n67) );
  and2 U70 ( .A(n68), .B(n69), .Y(n62) );
  aoi21 U71 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n70), .Y(n69) );
  and2 U72 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n70) );
  aoi21 U73 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n71), .Y(n68) );
  and2 U74 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n71) );
  and2 U75 ( .A(n72), .B(n73), .Y(n48) );
  and2 U76 ( .A(n74), .B(n75), .Y(n73) );
  and2 U77 ( .A(n76), .B(n77), .Y(n75) );
  aoi21 U78 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n78), .Y(n77) );
  and2 U79 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n78) );
  aoi21 U80 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n79), .Y(n76) );
  and2 U81 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n79) );
  and2 U82 ( .A(n80), .B(n81), .Y(n74) );
  aoi21 U83 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n82), .Y(n81) );
  and2 U84 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n82) );
  aoi21 U85 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n83), .Y(n80) );
  and2 U86 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n83) );
  and2 U87 ( .A(n84), .B(n85), .Y(n72) );
  and2 U88 ( .A(n86), .B(n87), .Y(n85) );
  aoi21 U89 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n88), .Y(n87) );
  and2 U90 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n88) );
  aoi21 U91 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n89), .Y(n86) );
  and2 U92 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n89) );
  and2 U93 ( .A(n90), .B(n91), .Y(n84) );
  nand2 U94 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n91) );
  aoi21 U95 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n92), .Y(n90) );
  and2 U96 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n92) );
  mux2 U97 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n93) );
  mux2 U98 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n94) );
  mux2 U99 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n95) );
  mux2 U100 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n96) );
  mux2 U101 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n97) );
  mux2 U102 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n98) );
  mux2 U103 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n99) );
  mux2 U104 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n100) );
  mux2 U105 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n101) );
  mux2 U106 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n102) );
  mux2 U107 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n103) );
  mux2 U108 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n104) );
  mux2 U109 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n105) );
  mux2 U110 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n106) );
  mux2 U111 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n107) );
  mux2 U112 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n108) );
  mux2 U113 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n109) );
  mux2 U114 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n110) );
  mux2 U115 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n111) );
  mux2 U116 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n112) );
  mux2 U117 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n113) );
  mux2 U118 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n114) );
  mux2 U119 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n115) );
  mux2 U120 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n116) );
  mux2 U121 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n117) );
  mux2 U122 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n118) );
  mux2 U123 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n119) );
  mux2 U124 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n120) );
  mux2 U125 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n121) );
  mux2 U126 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n122) );
  mux2 U127 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n123) );
endmodule


module pc_12 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_12 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_12 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_12 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_12 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__36 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__35 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__34 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_12 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_12 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_12 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_12 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_12 regfile ( .clk(clk), .rd_sel(rd_sel), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), 
        .rs2_rdata(rs2_rdata), .rf_data({rf_data[31:1], 
        SYNOPSYS_UNCONNECTED__0}) );
  pc_12 pc_ ( .clk(clk), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_12 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_12 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_12 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_12 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__36 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__35 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__34 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
endmodule


module mem_mux_11 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_11 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_11 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_11 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_11 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_11 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_11 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_11 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_11 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__33 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__32 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__31 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_11 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_11 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_11 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_11 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_11 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_11 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_11 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_11 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_11 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_11 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__33 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__32 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__31 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_10 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_10 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_10 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_10 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_10 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_10 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_10 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_10 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_10 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__30 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__29 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__28 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_10 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_10 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_10 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_10 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_10 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_10 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_10 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_10 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_10 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_10 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__30 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__29 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__28 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_9 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_9 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_9 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_9 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_9 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_9 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_9 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_9 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_9 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__27 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__26 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__25 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_9 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_9 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_9 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_9 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_9 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_9 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_9 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_9 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_9 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_9 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__27 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__26 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__25 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_8 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_8 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_8 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_8 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_8 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_8 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_8 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_8 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_8 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__24 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__23 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__22 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_8 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_8 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_8 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_8 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_8 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_8 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_8 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_8 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_8 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_8 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__24 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__23 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__22 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_7 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_7 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_7 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_7 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_7 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_7 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_7 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_7 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_7 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__21 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__20 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__19 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_7 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_7 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_7 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_7 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_7 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_7 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_7 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_7 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_7 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_7 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__21 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__20 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__19 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_6 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_6 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_6 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_6 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_6 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_6 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_6 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_6 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_6 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__18 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__17 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__16 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_6 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_6 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_6 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_6 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_6 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_6 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_6 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_6 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_6 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_6 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__18 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__17 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__16 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_5 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_5 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_5 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_5 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_5 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_5 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_5 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_5 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_5 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__15 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__14 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__13 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_5 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_5 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_5 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_5 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_5 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_5 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_5 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_5 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_5 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_5 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__15 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__14 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__13 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_4 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_4 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_4 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_4 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_4 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_4 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_4 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_4 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5, n6;
  assign shift_out[0] = alu_mux_1_out;

  \buf  U1 ( .A(shift_dir), .Y(n1) );
  mux2 U2 ( .A(shift_out[4]), .B(n2), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U3 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(n1), 
        .Y(n2) );
  mux2 U4 ( .A(shift_out[3]), .B(n3), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U5 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(n1), 
        .Y(n3) );
  mux2 U6 ( .A(shift_out[2]), .B(n4), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U7 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(n1), 
        .Y(n4) );
  mux2 U8 ( .A(shift_out[1]), .B(n5), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U9 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(n1), 
        .Y(n5) );
  mux2 U10 ( .A(alu_mux_1_out), .B(n6), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U11 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(n1), 
        .Y(n6) );
endmodule


module cmp_4 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__12 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__11 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__10 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_4 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1, n2;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_4 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_4 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_4 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_4 regfile ( .clk(n2), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_4 pc_ ( .clk(n2), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_4 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_4 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_4 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(n1), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_4 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__12 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__11 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__10 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(shift_dir), .Y(n1) );
  \buf  U2 ( .A(clk), .Y(n2) );
endmodule


module mem_mux_3 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_3 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_3 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_3 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_3 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_3 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_3 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_3 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_3 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__9 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__8 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__7 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_3 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_3 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_3 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_3 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_3 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_3 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_3 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_3 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_3 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_3 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__9 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__8 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__7 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module mem_mux_2 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_2 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_2 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_2 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_2 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_2 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_2 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_2 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_2 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__6 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__5 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__4 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_2 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_2 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_2 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_2 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_2 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_2 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_2 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_2 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_2 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_2 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__6 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__5 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__4 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module mem_mux_1 ( lb, lh, lw, lbu, lhu, mem_mux_sel, mem_mux_out );
  input [2:0] mem_mux_sel;
  input lb, lh, lw, lbu, lhu;
  output mem_mux_out;
  wire   n1, n2, n3, n4, n5, n6;

  inv U1 ( .A(n1), .Y(mem_mux_out) );
  aoi21 U2 ( .A(mem_mux_sel[1]), .B(lw), .C(n2), .Y(n1) );
  mux2 U3 ( .A(n3), .B(n4), .S0(mem_mux_sel[2]), .Y(n2) );
  mux2 U4 ( .A(lbu), .B(lhu), .S0(mem_mux_sel[0]), .Y(n4) );
  mux2 U5 ( .A(n5), .B(lh), .S0(mem_mux_sel[0]), .Y(n3) );
  and2 U6 ( .A(lb), .B(n6), .Y(n5) );
  inv U7 ( .A(mem_mux_sel[1]), .Y(n6) );
endmodule


module rd_mux_1 ( alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out, 
        rd_mux_sel, rd_mux_out );
  input [2:0] rd_mux_sel;
  input alu_out, shift_out, cmp_out, imm, pcp4, mem_mux_out;
  output rd_mux_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(rd_mux_out) );
  aoi21 U2 ( .A(rd_mux_sel[1]), .B(n2), .C(n3), .Y(n1) );
  mux2 U3 ( .A(n4), .B(n5), .S0(rd_mux_sel[2]), .Y(n3) );
  mux2 U4 ( .A(pcp4), .B(mem_mux_out), .S0(rd_mux_sel[0]), .Y(n5) );
  and2 U5 ( .A(n6), .B(n7), .Y(n4) );
  inv U6 ( .A(rd_mux_sel[1]), .Y(n7) );
  mux2 U7 ( .A(alu_out), .B(shift_out), .S0(rd_mux_sel[0]), .Y(n6) );
  mux2 U8 ( .A(cmp_out), .B(imm), .S0(rd_mux_sel[0]), .Y(n2) );
endmodule


module pcadder_1 ( pc, pc_adder_4, pc_cin, pc_cout, pcp4 );
  input pc, pc_adder_4, pc_cin;
  output pc_cout, pcp4;
  wire   n1, n2, n3;

  xor2 U1 ( .A(pc_cin), .B(n1), .Y(pcp4) );
  inv U2 ( .A(n2), .Y(pc_cout) );
  aoi21 U3 ( .A(pc_adder_4), .B(pc), .C(n3), .Y(n2) );
  and2 U4 ( .A(pc_cin), .B(n1), .Y(n3) );
  xor2 U5 ( .A(pc), .B(pc_adder_4), .Y(n1) );
endmodule


module regfile_1 ( clk, rd_sel, rs1_sel, rs2_sel, rd_mux_out, rs1_rdata, 
        rs2_rdata, rf_data );
  input [31:0] rd_sel;
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  output [31:0] rf_data;
  input clk, rd_mux_out;
  output rs1_rdata, rs2_rdata;
  wire   n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121;
  assign rf_data[0] = 1'b0;

  dff \rf_data_reg[31]  ( .D(n91), .CLK(clk), .Q(rf_data[31]) );
  dff \rf_data_reg[30]  ( .D(n92), .CLK(clk), .Q(rf_data[30]) );
  dff \rf_data_reg[29]  ( .D(n93), .CLK(clk), .Q(rf_data[29]) );
  dff \rf_data_reg[28]  ( .D(n94), .CLK(clk), .Q(rf_data[28]) );
  dff \rf_data_reg[27]  ( .D(n95), .CLK(clk), .Q(rf_data[27]) );
  dff \rf_data_reg[26]  ( .D(n96), .CLK(clk), .Q(rf_data[26]) );
  dff \rf_data_reg[25]  ( .D(n97), .CLK(clk), .Q(rf_data[25]) );
  dff \rf_data_reg[24]  ( .D(n98), .CLK(clk), .Q(rf_data[24]) );
  dff \rf_data_reg[23]  ( .D(n99), .CLK(clk), .Q(rf_data[23]) );
  dff \rf_data_reg[22]  ( .D(n100), .CLK(clk), .Q(rf_data[22]) );
  dff \rf_data_reg[21]  ( .D(n101), .CLK(clk), .Q(rf_data[21]) );
  dff \rf_data_reg[20]  ( .D(n102), .CLK(clk), .Q(rf_data[20]) );
  dff \rf_data_reg[19]  ( .D(n103), .CLK(clk), .Q(rf_data[19]) );
  dff \rf_data_reg[18]  ( .D(n104), .CLK(clk), .Q(rf_data[18]) );
  dff \rf_data_reg[17]  ( .D(n105), .CLK(clk), .Q(rf_data[17]) );
  dff \rf_data_reg[16]  ( .D(n106), .CLK(clk), .Q(rf_data[16]) );
  dff \rf_data_reg[15]  ( .D(n107), .CLK(clk), .Q(rf_data[15]) );
  dff \rf_data_reg[14]  ( .D(n108), .CLK(clk), .Q(rf_data[14]) );
  dff \rf_data_reg[13]  ( .D(n109), .CLK(clk), .Q(rf_data[13]) );
  dff \rf_data_reg[12]  ( .D(n110), .CLK(clk), .Q(rf_data[12]) );
  dff \rf_data_reg[11]  ( .D(n111), .CLK(clk), .Q(rf_data[11]) );
  dff \rf_data_reg[10]  ( .D(n112), .CLK(clk), .Q(rf_data[10]) );
  dff \rf_data_reg[9]  ( .D(n113), .CLK(clk), .Q(rf_data[9]) );
  dff \rf_data_reg[8]  ( .D(n114), .CLK(clk), .Q(rf_data[8]) );
  dff \rf_data_reg[7]  ( .D(n115), .CLK(clk), .Q(rf_data[7]) );
  dff \rf_data_reg[6]  ( .D(n116), .CLK(clk), .Q(rf_data[6]) );
  dff \rf_data_reg[5]  ( .D(n117), .CLK(clk), .Q(rf_data[5]) );
  dff \rf_data_reg[4]  ( .D(n118), .CLK(clk), .Q(rf_data[4]) );
  dff \rf_data_reg[3]  ( .D(n119), .CLK(clk), .Q(rf_data[3]) );
  dff \rf_data_reg[2]  ( .D(n120), .CLK(clk), .Q(rf_data[2]) );
  dff \rf_data_reg[1]  ( .D(n121), .CLK(clk), .Q(rf_data[1]) );
  nand2 U3 ( .A(n1), .B(n2), .Y(rs2_rdata) );
  and2 U4 ( .A(n3), .B(n4), .Y(n2) );
  and2 U5 ( .A(n5), .B(n6), .Y(n4) );
  and2 U6 ( .A(n7), .B(n8), .Y(n6) );
  aoi21 U7 ( .A(rs2_sel[11]), .B(rf_data[11]), .C(n9), .Y(n8) );
  and2 U8 ( .A(rs2_sel[10]), .B(rf_data[10]), .Y(n9) );
  aoi21 U9 ( .A(rs2_sel[13]), .B(rf_data[13]), .C(n10), .Y(n7) );
  and2 U10 ( .A(rs2_sel[12]), .B(rf_data[12]), .Y(n10) );
  and2 U11 ( .A(n11), .B(n12), .Y(n5) );
  aoi21 U12 ( .A(rs2_sel[15]), .B(rf_data[15]), .C(n13), .Y(n12) );
  and2 U13 ( .A(rs2_sel[14]), .B(rf_data[14]), .Y(n13) );
  aoi21 U14 ( .A(rs2_sel[17]), .B(rf_data[17]), .C(n14), .Y(n11) );
  and2 U15 ( .A(rs2_sel[16]), .B(rf_data[16]), .Y(n14) );
  and2 U16 ( .A(n15), .B(n16), .Y(n3) );
  and2 U17 ( .A(n17), .B(n18), .Y(n16) );
  aoi21 U18 ( .A(rs2_sel[19]), .B(rf_data[19]), .C(n19), .Y(n18) );
  and2 U19 ( .A(rs2_sel[18]), .B(rf_data[18]), .Y(n19) );
  aoi21 U20 ( .A(rs2_sel[20]), .B(rf_data[20]), .C(n20), .Y(n17) );
  and2 U21 ( .A(rs2_sel[1]), .B(rf_data[1]), .Y(n20) );
  and2 U22 ( .A(n21), .B(n22), .Y(n15) );
  aoi21 U23 ( .A(rs2_sel[22]), .B(rf_data[22]), .C(n23), .Y(n22) );
  and2 U24 ( .A(rs2_sel[21]), .B(rf_data[21]), .Y(n23) );
  aoi21 U25 ( .A(rs2_sel[24]), .B(rf_data[24]), .C(n24), .Y(n21) );
  and2 U26 ( .A(rs2_sel[23]), .B(rf_data[23]), .Y(n24) );
  and2 U27 ( .A(n25), .B(n26), .Y(n1) );
  and2 U28 ( .A(n27), .B(n28), .Y(n26) );
  and2 U29 ( .A(n29), .B(n30), .Y(n28) );
  aoi21 U30 ( .A(rs2_sel[26]), .B(rf_data[26]), .C(n31), .Y(n30) );
  and2 U31 ( .A(rs2_sel[25]), .B(rf_data[25]), .Y(n31) );
  aoi21 U32 ( .A(rs2_sel[28]), .B(rf_data[28]), .C(n32), .Y(n29) );
  and2 U33 ( .A(rs2_sel[27]), .B(rf_data[27]), .Y(n32) );
  and2 U34 ( .A(n33), .B(n34), .Y(n27) );
  aoi21 U35 ( .A(rs2_sel[2]), .B(rf_data[2]), .C(n35), .Y(n34) );
  and2 U36 ( .A(rs2_sel[29]), .B(rf_data[29]), .Y(n35) );
  aoi21 U37 ( .A(rs2_sel[31]), .B(rf_data[31]), .C(n36), .Y(n33) );
  and2 U38 ( .A(rs2_sel[30]), .B(rf_data[30]), .Y(n36) );
  and2 U39 ( .A(n37), .B(n38), .Y(n25) );
  and2 U40 ( .A(n39), .B(n40), .Y(n38) );
  aoi21 U41 ( .A(rs2_sel[4]), .B(rf_data[4]), .C(n41), .Y(n40) );
  and2 U42 ( .A(rs2_sel[3]), .B(rf_data[3]), .Y(n41) );
  aoi21 U43 ( .A(rs2_sel[6]), .B(rf_data[6]), .C(n42), .Y(n39) );
  and2 U44 ( .A(rs2_sel[5]), .B(rf_data[5]), .Y(n42) );
  and2 U45 ( .A(n43), .B(n44), .Y(n37) );
  nand2 U46 ( .A(rs2_sel[9]), .B(rf_data[9]), .Y(n44) );
  aoi21 U47 ( .A(rs2_sel[8]), .B(rf_data[8]), .C(n45), .Y(n43) );
  and2 U48 ( .A(rs2_sel[7]), .B(rf_data[7]), .Y(n45) );
  nand2 U49 ( .A(n46), .B(n47), .Y(rs1_rdata) );
  and2 U50 ( .A(n48), .B(n49), .Y(n47) );
  and2 U51 ( .A(n50), .B(n51), .Y(n49) );
  and2 U52 ( .A(n52), .B(n53), .Y(n51) );
  aoi21 U53 ( .A(rs1_sel[11]), .B(rf_data[11]), .C(n54), .Y(n53) );
  and2 U54 ( .A(rs1_sel[10]), .B(rf_data[10]), .Y(n54) );
  aoi21 U55 ( .A(rs1_sel[13]), .B(rf_data[13]), .C(n55), .Y(n52) );
  and2 U56 ( .A(rs1_sel[12]), .B(rf_data[12]), .Y(n55) );
  and2 U57 ( .A(n56), .B(n57), .Y(n50) );
  aoi21 U58 ( .A(rs1_sel[15]), .B(rf_data[15]), .C(n58), .Y(n57) );
  and2 U59 ( .A(rs1_sel[14]), .B(rf_data[14]), .Y(n58) );
  aoi21 U60 ( .A(rs1_sel[17]), .B(rf_data[17]), .C(n59), .Y(n56) );
  and2 U61 ( .A(rs1_sel[16]), .B(rf_data[16]), .Y(n59) );
  and2 U62 ( .A(n60), .B(n61), .Y(n48) );
  and2 U63 ( .A(n62), .B(n63), .Y(n61) );
  aoi21 U64 ( .A(rs1_sel[19]), .B(rf_data[19]), .C(n64), .Y(n63) );
  and2 U65 ( .A(rs1_sel[18]), .B(rf_data[18]), .Y(n64) );
  aoi21 U66 ( .A(rs1_sel[20]), .B(rf_data[20]), .C(n65), .Y(n62) );
  and2 U67 ( .A(rs1_sel[1]), .B(rf_data[1]), .Y(n65) );
  and2 U68 ( .A(n66), .B(n67), .Y(n60) );
  aoi21 U69 ( .A(rs1_sel[22]), .B(rf_data[22]), .C(n68), .Y(n67) );
  and2 U70 ( .A(rs1_sel[21]), .B(rf_data[21]), .Y(n68) );
  aoi21 U71 ( .A(rs1_sel[24]), .B(rf_data[24]), .C(n69), .Y(n66) );
  and2 U72 ( .A(rs1_sel[23]), .B(rf_data[23]), .Y(n69) );
  and2 U73 ( .A(n70), .B(n71), .Y(n46) );
  and2 U74 ( .A(n72), .B(n73), .Y(n71) );
  and2 U75 ( .A(n74), .B(n75), .Y(n73) );
  aoi21 U76 ( .A(rs1_sel[26]), .B(rf_data[26]), .C(n76), .Y(n75) );
  and2 U77 ( .A(rs1_sel[25]), .B(rf_data[25]), .Y(n76) );
  aoi21 U78 ( .A(rs1_sel[28]), .B(rf_data[28]), .C(n77), .Y(n74) );
  and2 U79 ( .A(rs1_sel[27]), .B(rf_data[27]), .Y(n77) );
  and2 U80 ( .A(n78), .B(n79), .Y(n72) );
  aoi21 U81 ( .A(rs1_sel[2]), .B(rf_data[2]), .C(n80), .Y(n79) );
  and2 U82 ( .A(rs1_sel[29]), .B(rf_data[29]), .Y(n80) );
  aoi21 U83 ( .A(rs1_sel[31]), .B(rf_data[31]), .C(n81), .Y(n78) );
  and2 U84 ( .A(rs1_sel[30]), .B(rf_data[30]), .Y(n81) );
  and2 U85 ( .A(n82), .B(n83), .Y(n70) );
  and2 U86 ( .A(n84), .B(n85), .Y(n83) );
  aoi21 U87 ( .A(rs1_sel[4]), .B(rf_data[4]), .C(n86), .Y(n85) );
  and2 U88 ( .A(rs1_sel[3]), .B(rf_data[3]), .Y(n86) );
  aoi21 U89 ( .A(rs1_sel[6]), .B(rf_data[6]), .C(n87), .Y(n84) );
  and2 U90 ( .A(rs1_sel[5]), .B(rf_data[5]), .Y(n87) );
  and2 U91 ( .A(n88), .B(n89), .Y(n82) );
  nand2 U92 ( .A(rs1_sel[9]), .B(rf_data[9]), .Y(n89) );
  aoi21 U93 ( .A(rs1_sel[8]), .B(rf_data[8]), .C(n90), .Y(n88) );
  and2 U94 ( .A(rs1_sel[7]), .B(rf_data[7]), .Y(n90) );
  mux2 U95 ( .A(rf_data[31]), .B(rd_mux_out), .S0(rd_sel[31]), .Y(n91) );
  mux2 U96 ( .A(rf_data[30]), .B(rd_mux_out), .S0(rd_sel[30]), .Y(n92) );
  mux2 U97 ( .A(rf_data[29]), .B(rd_mux_out), .S0(rd_sel[29]), .Y(n93) );
  mux2 U98 ( .A(rf_data[28]), .B(rd_mux_out), .S0(rd_sel[28]), .Y(n94) );
  mux2 U99 ( .A(rf_data[27]), .B(rd_mux_out), .S0(rd_sel[27]), .Y(n95) );
  mux2 U100 ( .A(rf_data[26]), .B(rd_mux_out), .S0(rd_sel[26]), .Y(n96) );
  mux2 U101 ( .A(rf_data[25]), .B(rd_mux_out), .S0(rd_sel[25]), .Y(n97) );
  mux2 U102 ( .A(rf_data[24]), .B(rd_mux_out), .S0(rd_sel[24]), .Y(n98) );
  mux2 U103 ( .A(rf_data[23]), .B(rd_mux_out), .S0(rd_sel[23]), .Y(n99) );
  mux2 U104 ( .A(rf_data[22]), .B(rd_mux_out), .S0(rd_sel[22]), .Y(n100) );
  mux2 U105 ( .A(rf_data[21]), .B(rd_mux_out), .S0(rd_sel[21]), .Y(n101) );
  mux2 U106 ( .A(rf_data[20]), .B(rd_mux_out), .S0(rd_sel[20]), .Y(n102) );
  mux2 U107 ( .A(rf_data[19]), .B(rd_mux_out), .S0(rd_sel[19]), .Y(n103) );
  mux2 U108 ( .A(rf_data[18]), .B(rd_mux_out), .S0(rd_sel[18]), .Y(n104) );
  mux2 U109 ( .A(rf_data[17]), .B(rd_mux_out), .S0(rd_sel[17]), .Y(n105) );
  mux2 U110 ( .A(rf_data[16]), .B(rd_mux_out), .S0(rd_sel[16]), .Y(n106) );
  mux2 U111 ( .A(rf_data[15]), .B(rd_mux_out), .S0(rd_sel[15]), .Y(n107) );
  mux2 U112 ( .A(rf_data[14]), .B(rd_mux_out), .S0(rd_sel[14]), .Y(n108) );
  mux2 U113 ( .A(rf_data[13]), .B(rd_mux_out), .S0(rd_sel[13]), .Y(n109) );
  mux2 U114 ( .A(rf_data[12]), .B(rd_mux_out), .S0(rd_sel[12]), .Y(n110) );
  mux2 U115 ( .A(rf_data[11]), .B(rd_mux_out), .S0(rd_sel[11]), .Y(n111) );
  mux2 U116 ( .A(rf_data[10]), .B(rd_mux_out), .S0(rd_sel[10]), .Y(n112) );
  mux2 U117 ( .A(rf_data[9]), .B(rd_mux_out), .S0(rd_sel[9]), .Y(n113) );
  mux2 U118 ( .A(rf_data[8]), .B(rd_mux_out), .S0(rd_sel[8]), .Y(n114) );
  mux2 U119 ( .A(rf_data[7]), .B(rd_mux_out), .S0(rd_sel[7]), .Y(n115) );
  mux2 U120 ( .A(rf_data[6]), .B(rd_mux_out), .S0(rd_sel[6]), .Y(n116) );
  mux2 U121 ( .A(rf_data[5]), .B(rd_mux_out), .S0(rd_sel[5]), .Y(n117) );
  mux2 U122 ( .A(rf_data[4]), .B(rd_mux_out), .S0(rd_sel[4]), .Y(n118) );
  mux2 U123 ( .A(rf_data[3]), .B(rd_mux_out), .S0(rd_sel[3]), .Y(n119) );
  mux2 U124 ( .A(rf_data[2]), .B(rd_mux_out), .S0(rd_sel[2]), .Y(n120) );
  mux2 U125 ( .A(rf_data[1]), .B(rd_mux_out), .S0(rd_sel[1]), .Y(n121) );
endmodule


module pc_1 ( clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out, pc );
  input clk, rst, pc_reset_value, pc_mux_sel, pcp4, alu_out;
  output pc;
  wire   pc_next, n1;

  dff pc_reg ( .D(pc_next), .CLK(clk), .Q(pc) );
  mux2 U3 ( .A(n1), .B(pc_reset_value), .S0(rst), .Y(pc_next) );
  mux2 U4 ( .A(pcp4), .B(alu_out), .S0(pc_mux_sel), .Y(n1) );
endmodule


module rs2_inverter_1 ( rs2_rdata, alu_inv_rs2, rs2_after_inv );
  input rs2_rdata, alu_inv_rs2;
  output rs2_after_inv;


  xor2 U1 ( .A(rs2_rdata), .B(alu_inv_rs2), .Y(rs2_after_inv) );
endmodule


module alu_1 ( alu_mux_1_out, alu_mux_2_out, alu_cin, alu_cout, alu_op, 
        alu_out );
  input [1:0] alu_op;
  input alu_mux_1_out, alu_mux_2_out, alu_cin;
  output alu_cout, alu_out;
  wire   n1, n2, n3, n4, n5, n6, n7;

  inv U1 ( .A(n1), .Y(alu_cout) );
  aoi21 U2 ( .A(alu_mux_2_out), .B(alu_mux_1_out), .C(n2), .Y(n1) );
  and2 U3 ( .A(alu_cin), .B(n3), .Y(n2) );
  mux2 U4 ( .A(n4), .B(n5), .S0(alu_op[1]), .Y(alu_out) );
  mux2 U5 ( .A(alu_mux_1_out), .B(n6), .S0(n3), .Y(n5) );
  xor2 U6 ( .A(n7), .B(n3), .Y(n4) );
  xor2 U7 ( .A(alu_mux_1_out), .B(alu_mux_2_out), .Y(n3) );
  and2 U8 ( .A(alu_cin), .B(n6), .Y(n7) );
  inv U9 ( .A(alu_op[0]), .Y(n6) );
endmodule


module shift_1 ( alu_mux_1_out, shift_amount, shift_dir, shift_in_from_right, 
        shift_in_from_left, shift_out );
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  input alu_mux_1_out, shift_dir;
  wire   alu_mux_1_out, n1, n2, n3, n4, n5;
  assign shift_out[0] = alu_mux_1_out;

  mux2 U1 ( .A(shift_out[4]), .B(n1), .S0(shift_amount[4]), .Y(shift_out[5])
         );
  mux2 U2 ( .A(shift_in_from_right[4]), .B(shift_in_from_left[4]), .S0(
        shift_dir), .Y(n1) );
  mux2 U3 ( .A(shift_out[3]), .B(n2), .S0(shift_amount[3]), .Y(shift_out[4])
         );
  mux2 U4 ( .A(shift_in_from_right[3]), .B(shift_in_from_left[3]), .S0(
        shift_dir), .Y(n2) );
  mux2 U5 ( .A(shift_out[2]), .B(n3), .S0(shift_amount[2]), .Y(shift_out[3])
         );
  mux2 U6 ( .A(shift_in_from_right[2]), .B(shift_in_from_left[2]), .S0(
        shift_dir), .Y(n3) );
  mux2 U7 ( .A(shift_out[1]), .B(n4), .S0(shift_amount[1]), .Y(shift_out[2])
         );
  mux2 U8 ( .A(shift_in_from_right[1]), .B(shift_in_from_left[1]), .S0(
        shift_dir), .Y(n4) );
  mux2 U9 ( .A(alu_mux_1_out), .B(n5), .S0(shift_amount[0]), .Y(shift_out[1])
         );
  mux2 U10 ( .A(shift_in_from_right[0]), .B(shift_in_from_left[0]), .S0(
        shift_dir), .Y(n5) );
endmodule


module cmp_1 ( rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in, cmp_eq_out, 
        cmp_lt_out );
  input rs1_rdata, cmp_mux_out, cmp_eq_in, cmp_lt_in;
  output cmp_eq_out, cmp_lt_out;
  wire   n1, n2, n3, n4;

  inv U1 ( .A(n1), .Y(cmp_lt_out) );
  aoi21 U2 ( .A(n2), .B(cmp_mux_out), .C(cmp_lt_in), .Y(n1) );
  and2 U3 ( .A(cmp_eq_in), .B(n3), .Y(n2) );
  inv U4 ( .A(rs1_rdata), .Y(n3) );
  and2 U5 ( .A(n4), .B(cmp_eq_in), .Y(cmp_eq_out) );
  xnor2 U6 ( .A(cmp_mux_out), .B(rs1_rdata), .Y(n4) );
endmodule


module mux2__3 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__2 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module mux2__1 ( a, b, s, z );
  input a, b, s;
  output z;


  mux2 U1 ( .A(a), .B(b), .S0(s), .Y(z) );
endmodule


module bitslice_1 ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_op, shift_dir, cmp_mux_sel, pc_mux_sel, 
        mem_mux_sel, rd_mux_sel, cmp_out, imm, alu_out, alu_cin, alu_cout, lb, 
        lh, lw, lbu, lhu, pc_reset_value, pc_adder_4, pc_cin, pc_cout, pc, 
        shift_amount, shift_in_from_right, shift_in_from_left, shift_out, 
        alu_mux_2_out, rs2_rdata, cmp_eq_in, cmp_lt_in, cmp_eq_out, cmp_lt_out, 
        cmp_src_a, cmp_src_b, rf_data );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [4:0] shift_amount;
  input [4:0] shift_in_from_right;
  input [4:0] shift_in_from_left;
  output [5:0] shift_out;
  output [31:0] rf_data;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, shift_dir,
         cmp_mux_sel, pc_mux_sel, cmp_out, imm, alu_cin, lb, lh, lw, lbu, lhu,
         pc_reset_value, pc_adder_4, pc_cin, cmp_eq_in, cmp_lt_in;
  output alu_out, alu_cout, pc_cout, pc, alu_mux_2_out, rs2_rdata, cmp_eq_out,
         cmp_lt_out, cmp_src_a, cmp_src_b;
  wire   mem_mux_out, rd_mux_out, pcp4, rs2_after_inv, alu_mux_1_out, n1;
  wire   SYNOPSYS_UNCONNECTED__0;
  assign rf_data[0] = 1'b0;

  mem_mux_1 mem_mux ( .lb(lb), .lh(lh), .lw(lw), .lbu(lbu), .lhu(lhu), 
        .mem_mux_sel(mem_mux_sel), .mem_mux_out(mem_mux_out) );
  rd_mux_1 rd_mux ( .alu_out(alu_out), .shift_out(shift_out[5]), .cmp_out(
        cmp_out), .imm(imm), .pcp4(pcp4), .mem_mux_out(mem_mux_out), 
        .rd_mux_sel(rd_mux_sel), .rd_mux_out(rd_mux_out) );
  pcadder_1 pcadder ( .pc(pc), .pc_adder_4(pc_adder_4), .pc_cin(pc_cin), 
        .pc_cout(pc_cout), .pcp4(pcp4) );
  regfile_1 regfile ( .clk(n1), .rd_sel(rd_sel), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_mux_out(rd_mux_out), .rs1_rdata(cmp_src_a), .rs2_rdata(
        rs2_rdata), .rf_data({rf_data[31:1], SYNOPSYS_UNCONNECTED__0}) );
  pc_1 pc_ ( .clk(n1), .rst(rst), .pc_reset_value(pc_reset_value), 
        .pc_mux_sel(pc_mux_sel), .pcp4(pcp4), .alu_out(alu_out), .pc(pc) );
  rs2_inverter_1 rs2_inverter ( .rs2_rdata(rs2_rdata), .alu_inv_rs2(
        alu_inv_rs2), .rs2_after_inv(rs2_after_inv) );
  alu_1 alu ( .alu_mux_1_out(alu_mux_1_out), .alu_mux_2_out(alu_mux_2_out), 
        .alu_cin(alu_cin), .alu_cout(alu_cout), .alu_op(alu_op), .alu_out(
        alu_out) );
  shift_1 shift ( .alu_mux_1_out(alu_mux_1_out), .shift_amount(shift_amount), 
        .shift_dir(shift_dir), .shift_in_from_right(shift_in_from_right), 
        .shift_in_from_left(shift_in_from_left), .shift_out(shift_out) );
  cmp_1 cmp ( .rs1_rdata(cmp_src_a), .cmp_mux_out(cmp_src_b), .cmp_eq_in(
        cmp_eq_in), .cmp_lt_in(cmp_lt_in), .cmp_eq_out(cmp_eq_out), 
        .cmp_lt_out(cmp_lt_out) );
  mux2__3 alu_mux_1 ( .a(cmp_src_a), .b(pc), .s(alu_mux_1_sel), .z(
        alu_mux_1_out) );
  mux2__2 alu_mux_2 ( .a(rs2_after_inv), .b(imm), .s(alu_mux_2_sel), .z(
        alu_mux_2_out) );
  mux2__1 cmp_mux ( .a(rs2_rdata), .b(imm), .s(cmp_mux_sel), .z(cmp_src_b) );
  \buf  U1 ( .A(clk), .Y(n1) );
endmodule


module datapath ( clk, rst, rs1_sel, rs2_sel, rd_sel, alu_mux_1_sel, 
        alu_mux_2_sel, alu_inv_rs2, alu_cin, alu_op, shift_msb, shift_dir, 
        cmp_mux_sel, pc_mux_sel, mem_mux_sel, rd_mux_sel, cmp_out, imm, cmp_lt, 
        cmp_eq, cmp_a_31, cmp_b_31, imem_addr, dmem_addr, dmem_rdata, 
        dmem_wdata );
  input [31:0] rs1_sel;
  input [31:0] rs2_sel;
  input [31:0] rd_sel;
  input [1:0] alu_op;
  input [2:0] mem_mux_sel;
  input [2:0] rd_mux_sel;
  input [31:0] imm;
  output [31:0] imem_addr;
  output [31:0] dmem_addr;
  input [31:0] dmem_rdata;
  output [31:0] dmem_wdata;
  input clk, rst, alu_mux_1_sel, alu_mux_2_sel, alu_inv_rs2, alu_cin,
         shift_msb, shift_dir, cmp_mux_sel, pc_mux_sel, cmp_out;
  output cmp_lt, cmp_eq, cmp_a_31, cmp_b_31;
  wire   \shift_in_from_right[31][4] , \shift_in_from_right[30][4] ,
         \shift_in_from_right[29][4] , \shift_in_from_right[28][4] ,
         \shift_in_from_right[27][4] , \shift_in_from_right[26][4] ,
         \shift_in_from_right[25][4] , \shift_in_from_right[24][4] ,
         \shift_in_from_right[23][4] , \shift_in_from_right[22][4] ,
         \shift_in_from_right[21][4] , \shift_in_from_right[20][4] ,
         \shift_in_from_right[19][4] , \shift_in_from_right[18][4] ,
         \shift_in_from_right[17][4] , \shift_in_from_right[16][4] ,
         \shift_in_from_right[15][3] , \shift_in_from_right[14][3] ,
         \shift_in_from_right[13][3] , \shift_in_from_right[12][3] ,
         \shift_in_from_right[11][3] , \shift_in_from_right[10][3] ,
         \shift_in_from_right[9][3] , \shift_in_from_right[8][3] ,
         \shift_in_from_right[7][2] , \shift_in_from_right[6][2] ,
         \shift_in_from_right[5][2] , \shift_in_from_right[4][2] ,
         \shift_in_from_right[3][1] , \shift_in_from_right[2][1] ,
         \shift_in_from_right[1][0] , \shift_in_from_left[30][0] ,
         \shift_in_from_left[29][1] , \shift_in_from_left[29][0] ,
         \shift_in_from_left[28][1] , \shift_in_from_left[28][0] ,
         \shift_in_from_left[27][2] , \shift_in_from_left[27][1] ,
         \shift_in_from_left[27][0] , \shift_in_from_left[26][2] ,
         \shift_in_from_left[26][1] , \shift_in_from_left[26][0] ,
         \shift_in_from_left[25][2] , \shift_in_from_left[25][1] ,
         \shift_in_from_left[25][0] , \shift_in_from_left[24][2] ,
         \shift_in_from_left[24][1] , \shift_in_from_left[24][0] ,
         \shift_in_from_left[23][3] , \shift_in_from_left[23][2] ,
         \shift_in_from_left[23][1] , \shift_in_from_left[23][0] ,
         \shift_in_from_left[22][3] , \shift_in_from_left[22][2] ,
         \shift_in_from_left[22][1] , \shift_in_from_left[22][0] ,
         \shift_in_from_left[21][3] , \shift_in_from_left[21][2] ,
         \shift_in_from_left[21][1] , \shift_in_from_left[21][0] ,
         \shift_in_from_left[20][3] , \shift_in_from_left[20][2] ,
         \shift_in_from_left[20][1] , \shift_in_from_left[20][0] ,
         \shift_in_from_left[19][3] , \shift_in_from_left[19][2] ,
         \shift_in_from_left[19][1] , \shift_in_from_left[19][0] ,
         \shift_in_from_left[18][3] , \shift_in_from_left[18][2] ,
         \shift_in_from_left[18][1] , \shift_in_from_left[18][0] ,
         \shift_in_from_left[17][3] , \shift_in_from_left[17][2] ,
         \shift_in_from_left[17][1] , \shift_in_from_left[17][0] ,
         \shift_in_from_left[16][3] , \shift_in_from_left[16][2] ,
         \shift_in_from_left[16][1] , \shift_in_from_left[16][0] ,
         \shift_in_from_left[15][4] , \shift_in_from_left[15][3] ,
         \shift_in_from_left[15][2] , \shift_in_from_left[15][1] ,
         \shift_in_from_left[15][0] , \shift_in_from_left[14][4] ,
         \shift_in_from_left[14][3] , \shift_in_from_left[14][2] ,
         \shift_in_from_left[14][1] , \shift_in_from_left[14][0] ,
         \shift_in_from_left[13][4] , \shift_in_from_left[13][3] ,
         \shift_in_from_left[13][2] , \shift_in_from_left[13][1] ,
         \shift_in_from_left[13][0] , \shift_in_from_left[12][4] ,
         \shift_in_from_left[12][3] , \shift_in_from_left[12][2] ,
         \shift_in_from_left[12][1] , \shift_in_from_left[12][0] ,
         \shift_in_from_left[11][4] , \shift_in_from_left[11][3] ,
         \shift_in_from_left[11][2] , \shift_in_from_left[11][1] ,
         \shift_in_from_left[11][0] , \shift_in_from_left[10][4] ,
         \shift_in_from_left[10][3] , \shift_in_from_left[10][2] ,
         \shift_in_from_left[10][1] , \shift_in_from_left[10][0] ,
         \shift_in_from_left[9][4] , \shift_in_from_left[9][3] ,
         \shift_in_from_left[9][2] , \shift_in_from_left[9][1] ,
         \shift_in_from_left[9][0] , \shift_in_from_left[8][4] ,
         \shift_in_from_left[8][3] , \shift_in_from_left[8][2] ,
         \shift_in_from_left[8][1] , \shift_in_from_left[8][0] ,
         \shift_in_from_left[7][4] , \shift_in_from_left[7][3] ,
         \shift_in_from_left[7][2] , \shift_in_from_left[7][1] ,
         \shift_in_from_left[7][0] , \shift_in_from_left[6][4] ,
         \shift_in_from_left[6][3] , \shift_in_from_left[6][2] ,
         \shift_in_from_left[6][1] , \shift_in_from_left[6][0] ,
         \shift_in_from_left[5][4] , \shift_in_from_left[5][3] ,
         \shift_in_from_left[5][2] , \shift_in_from_left[5][1] ,
         \shift_in_from_left[5][0] , \shift_in_from_left[4][4] ,
         \shift_in_from_left[4][3] , \shift_in_from_left[4][2] ,
         \shift_in_from_left[4][1] , \shift_in_from_left[4][0] ,
         \shift_in_from_left[3][4] , \shift_in_from_left[3][3] ,
         \shift_in_from_left[3][2] , \shift_in_from_left[3][1] ,
         \shift_in_from_left[3][0] , \shift_in_from_left[2][4] ,
         \shift_in_from_left[2][3] , \shift_in_from_left[2][2] ,
         \shift_in_from_left[2][1] , \shift_in_from_left[2][0] ,
         \shift_in_from_left[1][4] , \shift_in_from_left[1][3] ,
         \shift_in_from_left[1][2] , \shift_in_from_left[1][1] ,
         \shift_in_from_left[1][0] , \shift_in_from_left[0][4] ,
         \shift_in_from_left[0][3] , \shift_in_from_left[0][2] ,
         \shift_in_from_left[0][1] , \shift_in_from_left[0][0] , n1, n2, n3,
         n4, n5, n6;
  wire   [32:1] b_alu_cin;
  wire   [32:1] b_pc_cin;
  wire   [31:1] b_cmp_eq_out;
  wire   [31:1] b_cmp_lt_out;
  wire   [31:0] alu_mux_2_out;
  wire   SYNOPSYS_UNCONNECTED__0, SYNOPSYS_UNCONNECTED__1, 
        SYNOPSYS_UNCONNECTED__2, SYNOPSYS_UNCONNECTED__3, 
        SYNOPSYS_UNCONNECTED__4, SYNOPSYS_UNCONNECTED__5, 
        SYNOPSYS_UNCONNECTED__6, SYNOPSYS_UNCONNECTED__7, 
        SYNOPSYS_UNCONNECTED__8, SYNOPSYS_UNCONNECTED__9, 
        SYNOPSYS_UNCONNECTED__10, SYNOPSYS_UNCONNECTED__11, 
        SYNOPSYS_UNCONNECTED__12, SYNOPSYS_UNCONNECTED__13, 
        SYNOPSYS_UNCONNECTED__14, SYNOPSYS_UNCONNECTED__15, 
        SYNOPSYS_UNCONNECTED__16, SYNOPSYS_UNCONNECTED__17, 
        SYNOPSYS_UNCONNECTED__18, SYNOPSYS_UNCONNECTED__19, 
        SYNOPSYS_UNCONNECTED__20, SYNOPSYS_UNCONNECTED__21, 
        SYNOPSYS_UNCONNECTED__22, SYNOPSYS_UNCONNECTED__23, 
        SYNOPSYS_UNCONNECTED__24, SYNOPSYS_UNCONNECTED__25, 
        SYNOPSYS_UNCONNECTED__26, SYNOPSYS_UNCONNECTED__27, 
        SYNOPSYS_UNCONNECTED__28, SYNOPSYS_UNCONNECTED__29, 
        SYNOPSYS_UNCONNECTED__30, SYNOPSYS_UNCONNECTED__31;

  bitslice_0 \bitslices[0].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(cmp_out), .imm(imm[0]), .alu_out(dmem_addr[0]), .alu_cin(
        alu_cin), .alu_cout(b_alu_cin[1]), .lb(dmem_rdata[0]), .lh(
        dmem_rdata[0]), .lw(dmem_rdata[0]), .lbu(dmem_rdata[0]), .lhu(
        dmem_rdata[0]), .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(1'b0), .pc_cout(b_pc_cin[1]), .pc(imem_addr[0]), .shift_amount(alu_mux_2_out[4:0]), 
        .shift_in_from_right({1'b0, 1'b0, 1'b0, 1'b0, 1'b0}), 
        .shift_in_from_left({\shift_in_from_left[0][4] , 
        \shift_in_from_left[0][3] , \shift_in_from_left[0][2] , 
        \shift_in_from_left[0][1] , \shift_in_from_left[0][0] }), .shift_out({
        SYNOPSYS_UNCONNECTED__0, \shift_in_from_right[16][4] , 
        \shift_in_from_right[8][3] , \shift_in_from_right[4][2] , 
        \shift_in_from_right[2][1] , \shift_in_from_right[1][0] }), 
        .alu_mux_2_out(alu_mux_2_out[0]), .rs2_rdata(dmem_wdata[0]), 
        .cmp_eq_in(b_cmp_eq_out[1]), .cmp_lt_in(b_cmp_lt_out[1]), .cmp_eq_out(
        cmp_eq), .cmp_lt_out(cmp_lt) );
  bitslice_31 \bitslices[1].bitslice  ( .clk(n6), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n3), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[1]), .alu_out(dmem_addr[1]), .alu_cin(
        b_alu_cin[1]), .alu_cout(b_alu_cin[2]), .lb(dmem_rdata[1]), .lh(
        dmem_rdata[1]), .lw(dmem_rdata[1]), .lbu(dmem_rdata[1]), .lhu(
        dmem_rdata[1]), .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(
        b_pc_cin[1]), .pc_cout(b_pc_cin[2]), .pc(imem_addr[1]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 1'b0, 1'b0, 1'b0, 
        \shift_in_from_right[1][0] }), .shift_in_from_left({
        \shift_in_from_left[1][4] , \shift_in_from_left[1][3] , 
        \shift_in_from_left[1][2] , \shift_in_from_left[1][1] , 
        \shift_in_from_left[1][0] }), .shift_out({SYNOPSYS_UNCONNECTED__1, 
        \shift_in_from_right[17][4] , \shift_in_from_right[9][3] , 
        \shift_in_from_right[5][2] , \shift_in_from_right[3][1] , 
        \shift_in_from_left[0][0] }), .alu_mux_2_out(alu_mux_2_out[1]), 
        .rs2_rdata(dmem_wdata[1]), .cmp_eq_in(b_cmp_eq_out[2]), .cmp_lt_in(
        b_cmp_lt_out[2]), .cmp_eq_out(b_cmp_eq_out[1]), .cmp_lt_out(
        b_cmp_lt_out[1]) );
  bitslice_30 \bitslices[2].bitslice  ( .clk(n6), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n3), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[2]), .alu_out(dmem_addr[2]), .alu_cin(
        b_alu_cin[2]), .alu_cout(b_alu_cin[3]), .lb(dmem_rdata[2]), .lh(
        dmem_rdata[2]), .lw(dmem_rdata[2]), .lbu(dmem_rdata[2]), .lhu(
        dmem_rdata[2]), .pc_reset_value(1'b0), .pc_adder_4(1'b1), .pc_cin(
        b_pc_cin[2]), .pc_cout(b_pc_cin[3]), .pc(imem_addr[2]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 1'b0, 1'b0, 
        \shift_in_from_right[2][1] , \shift_in_from_left[0][0] }), 
        .shift_in_from_left({\shift_in_from_left[2][4] , 
        \shift_in_from_left[2][3] , \shift_in_from_left[2][2] , 
        \shift_in_from_left[2][1] , \shift_in_from_left[2][0] }), .shift_out({
        SYNOPSYS_UNCONNECTED__2, \shift_in_from_right[18][4] , 
        \shift_in_from_right[10][3] , \shift_in_from_right[6][2] , 
        \shift_in_from_left[0][1] , \shift_in_from_left[1][0] }), 
        .alu_mux_2_out(alu_mux_2_out[2]), .rs2_rdata(dmem_wdata[2]), 
        .cmp_eq_in(b_cmp_eq_out[3]), .cmp_lt_in(b_cmp_lt_out[3]), .cmp_eq_out(
        b_cmp_eq_out[2]), .cmp_lt_out(b_cmp_lt_out[2]) );
  bitslice_29 \bitslices[3].bitslice  ( .clk(n6), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n3), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[3]), .alu_out(dmem_addr[3]), .alu_cin(
        b_alu_cin[3]), .alu_cout(b_alu_cin[4]), .lb(dmem_rdata[3]), .lh(
        dmem_rdata[3]), .lw(dmem_rdata[3]), .lbu(dmem_rdata[3]), .lhu(
        dmem_rdata[3]), .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(
        b_pc_cin[3]), .pc_cout(b_pc_cin[4]), .pc(imem_addr[3]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 1'b0, 1'b0, 
        \shift_in_from_right[3][1] , \shift_in_from_left[1][0] }), 
        .shift_in_from_left({\shift_in_from_left[3][4] , 
        \shift_in_from_left[3][3] , \shift_in_from_left[3][2] , 
        \shift_in_from_left[3][1] , \shift_in_from_left[3][0] }), .shift_out({
        SYNOPSYS_UNCONNECTED__3, \shift_in_from_right[19][4] , 
        \shift_in_from_right[11][3] , \shift_in_from_right[7][2] , 
        \shift_in_from_left[1][1] , \shift_in_from_left[2][0] }), 
        .alu_mux_2_out(alu_mux_2_out[3]), .rs2_rdata(dmem_wdata[3]), 
        .cmp_eq_in(b_cmp_eq_out[4]), .cmp_lt_in(b_cmp_lt_out[4]), .cmp_eq_out(
        b_cmp_eq_out[3]), .cmp_lt_out(b_cmp_lt_out[3]) );
  bitslice_28 \bitslices[4].bitslice  ( .clk(n6), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n3), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[4]), .alu_out(dmem_addr[4]), .alu_cin(
        b_alu_cin[4]), .alu_cout(b_alu_cin[5]), .lb(dmem_rdata[4]), .lh(
        dmem_rdata[4]), .lw(dmem_rdata[4]), .lbu(dmem_rdata[4]), .lhu(
        dmem_rdata[4]), .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(
        b_pc_cin[4]), .pc_cout(b_pc_cin[5]), .pc(imem_addr[4]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 1'b0, 
        \shift_in_from_right[4][2] , \shift_in_from_left[0][1] , 
        \shift_in_from_left[2][0] }), .shift_in_from_left({
        \shift_in_from_left[4][4] , \shift_in_from_left[4][3] , 
        \shift_in_from_left[4][2] , \shift_in_from_left[4][1] , 
        \shift_in_from_left[4][0] }), .shift_out({SYNOPSYS_UNCONNECTED__4, 
        \shift_in_from_right[20][4] , \shift_in_from_right[12][3] , 
        \shift_in_from_left[0][2] , \shift_in_from_left[2][1] , 
        \shift_in_from_left[3][0] }), .alu_mux_2_out(alu_mux_2_out[4]), 
        .rs2_rdata(dmem_wdata[4]), .cmp_eq_in(b_cmp_eq_out[5]), .cmp_lt_in(
        b_cmp_lt_out[5]), .cmp_eq_out(b_cmp_eq_out[4]), .cmp_lt_out(
        b_cmp_lt_out[4]) );
  bitslice_27 \bitslices[5].bitslice  ( .clk(n6), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n3), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[5]), .alu_out(dmem_addr[5]), .alu_cin(
        b_alu_cin[5]), .alu_cout(b_alu_cin[6]), .lb(dmem_rdata[5]), .lh(
        dmem_rdata[5]), .lw(dmem_rdata[5]), .lbu(dmem_rdata[5]), .lhu(
        dmem_rdata[5]), .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(
        b_pc_cin[5]), .pc_cout(b_pc_cin[6]), .pc(imem_addr[5]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 1'b0, 
        \shift_in_from_right[5][2] , \shift_in_from_left[1][1] , 
        \shift_in_from_left[3][0] }), .shift_in_from_left({
        \shift_in_from_left[5][4] , \shift_in_from_left[5][3] , 
        \shift_in_from_left[5][2] , \shift_in_from_left[5][1] , 
        \shift_in_from_left[5][0] }), .shift_out({SYNOPSYS_UNCONNECTED__5, 
        \shift_in_from_right[21][4] , \shift_in_from_right[13][3] , 
        \shift_in_from_left[1][2] , \shift_in_from_left[3][1] , 
        \shift_in_from_left[4][0] }), .rs2_rdata(dmem_wdata[5]), .cmp_eq_in(
        b_cmp_eq_out[6]), .cmp_lt_in(b_cmp_lt_out[6]), .cmp_eq_out(
        b_cmp_eq_out[5]), .cmp_lt_out(b_cmp_lt_out[5]) );
  bitslice_26 \bitslices[6].bitslice  ( .clk(n6), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n3), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[6]), .alu_out(dmem_addr[6]), .alu_cin(
        b_alu_cin[6]), .alu_cout(b_alu_cin[7]), .lb(dmem_rdata[6]), .lh(
        dmem_rdata[6]), .lw(dmem_rdata[6]), .lbu(dmem_rdata[6]), .lhu(
        dmem_rdata[6]), .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(
        b_pc_cin[6]), .pc_cout(b_pc_cin[7]), .pc(imem_addr[6]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 1'b0, 
        \shift_in_from_right[6][2] , \shift_in_from_left[2][1] , 
        \shift_in_from_left[4][0] }), .shift_in_from_left({
        \shift_in_from_left[6][4] , \shift_in_from_left[6][3] , 
        \shift_in_from_left[6][2] , \shift_in_from_left[6][1] , 
        \shift_in_from_left[6][0] }), .shift_out({SYNOPSYS_UNCONNECTED__6, 
        \shift_in_from_right[22][4] , \shift_in_from_right[14][3] , 
        \shift_in_from_left[2][2] , \shift_in_from_left[4][1] , 
        \shift_in_from_left[5][0] }), .rs2_rdata(dmem_wdata[6]), .cmp_eq_in(
        b_cmp_eq_out[7]), .cmp_lt_in(b_cmp_lt_out[7]), .cmp_eq_out(
        b_cmp_eq_out[6]), .cmp_lt_out(b_cmp_lt_out[6]) );
  bitslice_25 \bitslices[7].bitslice  ( .clk(n6), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n3), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[7]), .alu_out(dmem_addr[7]), .alu_cin(
        b_alu_cin[7]), .alu_cout(b_alu_cin[8]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[7]), .lw(dmem_rdata[7]), .lbu(dmem_rdata[7]), .lhu(
        dmem_rdata[7]), .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(
        b_pc_cin[7]), .pc_cout(b_pc_cin[8]), .pc(imem_addr[7]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 1'b0, 
        \shift_in_from_right[7][2] , \shift_in_from_left[3][1] , 
        \shift_in_from_left[5][0] }), .shift_in_from_left({
        \shift_in_from_left[7][4] , \shift_in_from_left[7][3] , 
        \shift_in_from_left[7][2] , \shift_in_from_left[7][1] , 
        \shift_in_from_left[7][0] }), .shift_out({SYNOPSYS_UNCONNECTED__7, 
        \shift_in_from_right[23][4] , \shift_in_from_right[15][3] , 
        \shift_in_from_left[3][2] , \shift_in_from_left[5][1] , 
        \shift_in_from_left[6][0] }), .rs2_rdata(dmem_wdata[7]), .cmp_eq_in(
        b_cmp_eq_out[8]), .cmp_lt_in(b_cmp_lt_out[8]), .cmp_eq_out(
        b_cmp_eq_out[7]), .cmp_lt_out(b_cmp_lt_out[7]) );
  bitslice_24 \bitslices[8].bitslice  ( .clk(n6), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n3), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[8]), .alu_out(dmem_addr[8]), .alu_cin(
        b_alu_cin[8]), .alu_cout(b_alu_cin[9]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[8]), .lw(dmem_rdata[8]), .lbu(1'b0), .lhu(dmem_rdata[8]), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[8]), 
        .pc_cout(b_pc_cin[9]), .pc(imem_addr[8]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 
        \shift_in_from_right[8][3] , \shift_in_from_left[0][2] , 
        \shift_in_from_left[4][1] , \shift_in_from_left[6][0] }), 
        .shift_in_from_left({\shift_in_from_left[8][4] , 
        \shift_in_from_left[8][3] , \shift_in_from_left[8][2] , 
        \shift_in_from_left[8][1] , \shift_in_from_left[8][0] }), .shift_out({
        SYNOPSYS_UNCONNECTED__8, \shift_in_from_right[24][4] , 
        \shift_in_from_left[0][3] , \shift_in_from_left[4][2] , 
        \shift_in_from_left[6][1] , \shift_in_from_left[7][0] }), .rs2_rdata(
        dmem_wdata[8]), .cmp_eq_in(b_cmp_eq_out[9]), .cmp_lt_in(
        b_cmp_lt_out[9]), .cmp_eq_out(b_cmp_eq_out[8]), .cmp_lt_out(
        b_cmp_lt_out[8]) );
  bitslice_23 \bitslices[9].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[9]), .alu_out(dmem_addr[9]), .alu_cin(
        b_alu_cin[9]), .alu_cout(b_alu_cin[10]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[9]), .lw(dmem_rdata[9]), .lbu(1'b0), .lhu(dmem_rdata[9]), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[9]), 
        .pc_cout(b_pc_cin[10]), .pc(imem_addr[9]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 
        \shift_in_from_right[9][3] , \shift_in_from_left[1][2] , 
        \shift_in_from_left[5][1] , \shift_in_from_left[7][0] }), 
        .shift_in_from_left({\shift_in_from_left[9][4] , 
        \shift_in_from_left[9][3] , \shift_in_from_left[9][2] , 
        \shift_in_from_left[9][1] , \shift_in_from_left[9][0] }), .shift_out({
        SYNOPSYS_UNCONNECTED__9, \shift_in_from_right[25][4] , 
        \shift_in_from_left[1][3] , \shift_in_from_left[5][2] , 
        \shift_in_from_left[7][1] , \shift_in_from_left[8][0] }), .rs2_rdata(
        dmem_wdata[9]), .cmp_eq_in(b_cmp_eq_out[10]), .cmp_lt_in(
        b_cmp_lt_out[10]), .cmp_eq_out(b_cmp_eq_out[9]), .cmp_lt_out(
        b_cmp_lt_out[9]) );
  bitslice_22 \bitslices[10].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[10]), .alu_out(dmem_addr[10]), .alu_cin(
        b_alu_cin[10]), .alu_cout(b_alu_cin[11]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[10]), .lw(dmem_rdata[10]), .lbu(1'b0), .lhu(dmem_rdata[10]), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[10]), 
        .pc_cout(b_pc_cin[11]), .pc(imem_addr[10]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 
        \shift_in_from_right[10][3] , \shift_in_from_left[2][2] , 
        \shift_in_from_left[6][1] , \shift_in_from_left[8][0] }), 
        .shift_in_from_left({\shift_in_from_left[10][4] , 
        \shift_in_from_left[10][3] , \shift_in_from_left[10][2] , 
        \shift_in_from_left[10][1] , \shift_in_from_left[10][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__10, \shift_in_from_right[26][4] , 
        \shift_in_from_left[2][3] , \shift_in_from_left[6][2] , 
        \shift_in_from_left[8][1] , \shift_in_from_left[9][0] }), .rs2_rdata(
        dmem_wdata[10]), .cmp_eq_in(b_cmp_eq_out[11]), .cmp_lt_in(
        b_cmp_lt_out[11]), .cmp_eq_out(b_cmp_eq_out[10]), .cmp_lt_out(
        b_cmp_lt_out[10]) );
  bitslice_21 \bitslices[11].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[11]), .alu_out(dmem_addr[11]), .alu_cin(
        b_alu_cin[11]), .alu_cout(b_alu_cin[12]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[11]), .lw(dmem_rdata[11]), .lbu(1'b0), .lhu(dmem_rdata[11]), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[11]), 
        .pc_cout(b_pc_cin[12]), .pc(imem_addr[11]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 
        \shift_in_from_right[11][3] , \shift_in_from_left[3][2] , 
        \shift_in_from_left[7][1] , \shift_in_from_left[9][0] }), 
        .shift_in_from_left({\shift_in_from_left[11][4] , 
        \shift_in_from_left[11][3] , \shift_in_from_left[11][2] , 
        \shift_in_from_left[11][1] , \shift_in_from_left[11][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__11, \shift_in_from_right[27][4] , 
        \shift_in_from_left[3][3] , \shift_in_from_left[7][2] , 
        \shift_in_from_left[9][1] , \shift_in_from_left[10][0] }), .rs2_rdata(
        dmem_wdata[11]), .cmp_eq_in(b_cmp_eq_out[12]), .cmp_lt_in(
        b_cmp_lt_out[12]), .cmp_eq_out(b_cmp_eq_out[11]), .cmp_lt_out(
        b_cmp_lt_out[11]) );
  bitslice_20 \bitslices[12].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[12]), .alu_out(dmem_addr[12]), .alu_cin(
        b_alu_cin[12]), .alu_cout(b_alu_cin[13]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[12]), .lw(dmem_rdata[12]), .lbu(1'b0), .lhu(dmem_rdata[12]), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[12]), 
        .pc_cout(b_pc_cin[13]), .pc(imem_addr[12]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 
        \shift_in_from_right[12][3] , \shift_in_from_left[4][2] , 
        \shift_in_from_left[8][1] , \shift_in_from_left[10][0] }), 
        .shift_in_from_left({\shift_in_from_left[12][4] , 
        \shift_in_from_left[12][3] , \shift_in_from_left[12][2] , 
        \shift_in_from_left[12][1] , \shift_in_from_left[12][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__12, \shift_in_from_right[28][4] , 
        \shift_in_from_left[4][3] , \shift_in_from_left[8][2] , 
        \shift_in_from_left[10][1] , \shift_in_from_left[11][0] }), 
        .rs2_rdata(dmem_wdata[12]), .cmp_eq_in(b_cmp_eq_out[13]), .cmp_lt_in(
        b_cmp_lt_out[13]), .cmp_eq_out(b_cmp_eq_out[12]), .cmp_lt_out(
        b_cmp_lt_out[12]) );
  bitslice_19 \bitslices[13].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[13]), .alu_out(dmem_addr[13]), .alu_cin(
        b_alu_cin[13]), .alu_cout(b_alu_cin[14]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[13]), .lw(dmem_rdata[13]), .lbu(1'b0), .lhu(dmem_rdata[13]), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[13]), 
        .pc_cout(b_pc_cin[14]), .pc(imem_addr[13]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 
        \shift_in_from_right[13][3] , \shift_in_from_left[5][2] , 
        \shift_in_from_left[9][1] , \shift_in_from_left[11][0] }), 
        .shift_in_from_left({\shift_in_from_left[13][4] , 
        \shift_in_from_left[13][3] , \shift_in_from_left[13][2] , 
        \shift_in_from_left[13][1] , \shift_in_from_left[13][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__13, \shift_in_from_right[29][4] , 
        \shift_in_from_left[5][3] , \shift_in_from_left[9][2] , 
        \shift_in_from_left[11][1] , \shift_in_from_left[12][0] }), 
        .rs2_rdata(dmem_wdata[13]), .cmp_eq_in(b_cmp_eq_out[14]), .cmp_lt_in(
        b_cmp_lt_out[14]), .cmp_eq_out(b_cmp_eq_out[13]), .cmp_lt_out(
        b_cmp_lt_out[13]) );
  bitslice_18 \bitslices[14].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[14]), .alu_out(dmem_addr[14]), .alu_cin(
        b_alu_cin[14]), .alu_cout(b_alu_cin[15]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[14]), .lw(dmem_rdata[14]), .lbu(1'b0), .lhu(dmem_rdata[14]), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[14]), 
        .pc_cout(b_pc_cin[15]), .pc(imem_addr[14]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 
        \shift_in_from_right[14][3] , \shift_in_from_left[6][2] , 
        \shift_in_from_left[10][1] , \shift_in_from_left[12][0] }), 
        .shift_in_from_left({\shift_in_from_left[14][4] , 
        \shift_in_from_left[14][3] , \shift_in_from_left[14][2] , 
        \shift_in_from_left[14][1] , \shift_in_from_left[14][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__14, \shift_in_from_right[30][4] , 
        \shift_in_from_left[6][3] , \shift_in_from_left[10][2] , 
        \shift_in_from_left[12][1] , \shift_in_from_left[13][0] }), 
        .rs2_rdata(dmem_wdata[14]), .cmp_eq_in(b_cmp_eq_out[15]), .cmp_lt_in(
        b_cmp_lt_out[15]), .cmp_eq_out(b_cmp_eq_out[14]), .cmp_lt_out(
        b_cmp_lt_out[14]) );
  bitslice_17 \bitslices[15].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[15]), .alu_out(dmem_addr[15]), .alu_cin(
        b_alu_cin[15]), .alu_cout(b_alu_cin[16]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[15]), .lbu(1'b0), .lhu(dmem_rdata[15]), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[15]), 
        .pc_cout(b_pc_cin[16]), .pc(imem_addr[15]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({1'b0, 
        \shift_in_from_right[15][3] , \shift_in_from_left[7][2] , 
        \shift_in_from_left[11][1] , \shift_in_from_left[13][0] }), 
        .shift_in_from_left({\shift_in_from_left[15][4] , 
        \shift_in_from_left[15][3] , \shift_in_from_left[15][2] , 
        \shift_in_from_left[15][1] , \shift_in_from_left[15][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__15, \shift_in_from_right[31][4] , 
        \shift_in_from_left[7][3] , \shift_in_from_left[11][2] , 
        \shift_in_from_left[13][1] , \shift_in_from_left[14][0] }), 
        .rs2_rdata(dmem_wdata[15]), .cmp_eq_in(b_cmp_eq_out[16]), .cmp_lt_in(
        b_cmp_lt_out[16]), .cmp_eq_out(b_cmp_eq_out[15]), .cmp_lt_out(
        b_cmp_lt_out[15]) );
  bitslice_16 \bitslices[16].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[16]), .alu_out(dmem_addr[16]), .alu_cin(
        b_alu_cin[16]), .alu_cout(b_alu_cin[17]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[16]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[16]), 
        .pc_cout(b_pc_cin[17]), .pc(imem_addr[16]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[16][4] , \shift_in_from_left[0][3] , 
        \shift_in_from_left[8][2] , \shift_in_from_left[12][1] , 
        \shift_in_from_left[14][0] }), .shift_in_from_left({shift_msb, 
        \shift_in_from_left[16][3] , \shift_in_from_left[16][2] , 
        \shift_in_from_left[16][1] , \shift_in_from_left[16][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__16, \shift_in_from_left[0][4] , 
        \shift_in_from_left[8][3] , \shift_in_from_left[12][2] , 
        \shift_in_from_left[14][1] , \shift_in_from_left[15][0] }), 
        .rs2_rdata(dmem_wdata[16]), .cmp_eq_in(b_cmp_eq_out[17]), .cmp_lt_in(
        b_cmp_lt_out[17]), .cmp_eq_out(b_cmp_eq_out[16]), .cmp_lt_out(
        b_cmp_lt_out[16]) );
  bitslice_15 \bitslices[17].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[17]), .alu_out(dmem_addr[17]), .alu_cin(
        b_alu_cin[17]), .alu_cout(b_alu_cin[18]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[17]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[17]), 
        .pc_cout(b_pc_cin[18]), .pc(imem_addr[17]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[17][4] , \shift_in_from_left[1][3] , 
        \shift_in_from_left[9][2] , \shift_in_from_left[13][1] , 
        \shift_in_from_left[15][0] }), .shift_in_from_left({shift_msb, 
        \shift_in_from_left[17][3] , \shift_in_from_left[17][2] , 
        \shift_in_from_left[17][1] , \shift_in_from_left[17][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__17, \shift_in_from_left[1][4] , 
        \shift_in_from_left[9][3] , \shift_in_from_left[13][2] , 
        \shift_in_from_left[15][1] , \shift_in_from_left[16][0] }), 
        .rs2_rdata(dmem_wdata[17]), .cmp_eq_in(b_cmp_eq_out[18]), .cmp_lt_in(
        b_cmp_lt_out[18]), .cmp_eq_out(b_cmp_eq_out[17]), .cmp_lt_out(
        b_cmp_lt_out[17]) );
  bitslice_14 \bitslices[18].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[18]), .alu_out(dmem_addr[18]), .alu_cin(
        b_alu_cin[18]), .alu_cout(b_alu_cin[19]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[18]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[18]), 
        .pc_cout(b_pc_cin[19]), .pc(imem_addr[18]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[18][4] , \shift_in_from_left[2][3] , 
        \shift_in_from_left[10][2] , \shift_in_from_left[14][1] , 
        \shift_in_from_left[16][0] }), .shift_in_from_left({shift_msb, 
        \shift_in_from_left[18][3] , \shift_in_from_left[18][2] , 
        \shift_in_from_left[18][1] , \shift_in_from_left[18][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__18, \shift_in_from_left[2][4] , 
        \shift_in_from_left[10][3] , \shift_in_from_left[14][2] , 
        \shift_in_from_left[16][1] , \shift_in_from_left[17][0] }), 
        .rs2_rdata(dmem_wdata[18]), .cmp_eq_in(b_cmp_eq_out[19]), .cmp_lt_in(
        b_cmp_lt_out[19]), .cmp_eq_out(b_cmp_eq_out[18]), .cmp_lt_out(
        b_cmp_lt_out[18]) );
  bitslice_13 \bitslices[19].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[19]), .alu_out(dmem_addr[19]), .alu_cin(
        b_alu_cin[19]), .alu_cout(b_alu_cin[20]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[19]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[19]), 
        .pc_cout(b_pc_cin[20]), .pc(imem_addr[19]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[19][4] , \shift_in_from_left[3][3] , 
        \shift_in_from_left[11][2] , \shift_in_from_left[15][1] , 
        \shift_in_from_left[17][0] }), .shift_in_from_left({shift_msb, 
        \shift_in_from_left[19][3] , \shift_in_from_left[19][2] , 
        \shift_in_from_left[19][1] , \shift_in_from_left[19][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__19, \shift_in_from_left[3][4] , 
        \shift_in_from_left[11][3] , \shift_in_from_left[15][2] , 
        \shift_in_from_left[17][1] , \shift_in_from_left[18][0] }), 
        .rs2_rdata(dmem_wdata[19]), .cmp_eq_in(b_cmp_eq_out[20]), .cmp_lt_in(
        b_cmp_lt_out[20]), .cmp_eq_out(b_cmp_eq_out[19]), .cmp_lt_out(
        b_cmp_lt_out[19]) );
  bitslice_12 \bitslices[20].bitslice  ( .clk(n5), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n2), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[20]), .alu_out(dmem_addr[20]), .alu_cin(
        b_alu_cin[20]), .alu_cout(b_alu_cin[21]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[20]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[20]), 
        .pc_cout(b_pc_cin[21]), .pc(imem_addr[20]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[20][4] , \shift_in_from_left[4][3] , 
        \shift_in_from_left[12][2] , \shift_in_from_left[16][1] , 
        \shift_in_from_left[18][0] }), .shift_in_from_left({shift_msb, 
        \shift_in_from_left[20][3] , \shift_in_from_left[20][2] , 
        \shift_in_from_left[20][1] , \shift_in_from_left[20][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__20, \shift_in_from_left[4][4] , 
        \shift_in_from_left[12][3] , \shift_in_from_left[16][2] , 
        \shift_in_from_left[18][1] , \shift_in_from_left[19][0] }), 
        .rs2_rdata(dmem_wdata[20]), .cmp_eq_in(b_cmp_eq_out[21]), .cmp_lt_in(
        b_cmp_lt_out[21]), .cmp_eq_out(b_cmp_eq_out[20]), .cmp_lt_out(
        b_cmp_lt_out[20]) );
  bitslice_11 \bitslices[21].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[21]), .alu_out(dmem_addr[21]), .alu_cin(
        b_alu_cin[21]), .alu_cout(b_alu_cin[22]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[21]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[21]), 
        .pc_cout(b_pc_cin[22]), .pc(imem_addr[21]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[21][4] , \shift_in_from_left[5][3] , 
        \shift_in_from_left[13][2] , \shift_in_from_left[17][1] , 
        \shift_in_from_left[19][0] }), .shift_in_from_left({shift_msb, 
        \shift_in_from_left[21][3] , \shift_in_from_left[21][2] , 
        \shift_in_from_left[21][1] , \shift_in_from_left[21][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__21, \shift_in_from_left[5][4] , 
        \shift_in_from_left[13][3] , \shift_in_from_left[17][2] , 
        \shift_in_from_left[19][1] , \shift_in_from_left[20][0] }), 
        .rs2_rdata(dmem_wdata[21]), .cmp_eq_in(b_cmp_eq_out[22]), .cmp_lt_in(
        b_cmp_lt_out[22]), .cmp_eq_out(b_cmp_eq_out[21]), .cmp_lt_out(
        b_cmp_lt_out[21]) );
  bitslice_10 \bitslices[22].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[22]), .alu_out(dmem_addr[22]), .alu_cin(
        b_alu_cin[22]), .alu_cout(b_alu_cin[23]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[22]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[22]), 
        .pc_cout(b_pc_cin[23]), .pc(imem_addr[22]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[22][4] , \shift_in_from_left[6][3] , 
        \shift_in_from_left[14][2] , \shift_in_from_left[18][1] , 
        \shift_in_from_left[20][0] }), .shift_in_from_left({shift_msb, 
        \shift_in_from_left[22][3] , \shift_in_from_left[22][2] , 
        \shift_in_from_left[22][1] , \shift_in_from_left[22][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__22, \shift_in_from_left[6][4] , 
        \shift_in_from_left[14][3] , \shift_in_from_left[18][2] , 
        \shift_in_from_left[20][1] , \shift_in_from_left[21][0] }), 
        .rs2_rdata(dmem_wdata[22]), .cmp_eq_in(b_cmp_eq_out[23]), .cmp_lt_in(
        b_cmp_lt_out[23]), .cmp_eq_out(b_cmp_eq_out[22]), .cmp_lt_out(
        b_cmp_lt_out[22]) );
  bitslice_9 \bitslices[23].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[23]), .alu_out(dmem_addr[23]), .alu_cin(
        b_alu_cin[23]), .alu_cout(b_alu_cin[24]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[23]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[23]), 
        .pc_cout(b_pc_cin[24]), .pc(imem_addr[23]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[23][4] , \shift_in_from_left[7][3] , 
        \shift_in_from_left[15][2] , \shift_in_from_left[19][1] , 
        \shift_in_from_left[21][0] }), .shift_in_from_left({shift_msb, 
        \shift_in_from_left[23][3] , \shift_in_from_left[23][2] , 
        \shift_in_from_left[23][1] , \shift_in_from_left[23][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__23, \shift_in_from_left[7][4] , 
        \shift_in_from_left[15][3] , \shift_in_from_left[19][2] , 
        \shift_in_from_left[21][1] , \shift_in_from_left[22][0] }), 
        .rs2_rdata(dmem_wdata[23]), .cmp_eq_in(b_cmp_eq_out[24]), .cmp_lt_in(
        b_cmp_lt_out[24]), .cmp_eq_out(b_cmp_eq_out[23]), .cmp_lt_out(
        b_cmp_lt_out[23]) );
  bitslice_8 \bitslices[24].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[24]), .alu_out(dmem_addr[24]), .alu_cin(
        b_alu_cin[24]), .alu_cout(b_alu_cin[25]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[24]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[24]), 
        .pc_cout(b_pc_cin[25]), .pc(imem_addr[24]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[24][4] , \shift_in_from_left[8][3] , 
        \shift_in_from_left[16][2] , \shift_in_from_left[20][1] , 
        \shift_in_from_left[22][0] }), .shift_in_from_left({shift_msb, 
        shift_msb, \shift_in_from_left[24][2] , \shift_in_from_left[24][1] , 
        \shift_in_from_left[24][0] }), .shift_out({SYNOPSYS_UNCONNECTED__24, 
        \shift_in_from_left[8][4] , \shift_in_from_left[16][3] , 
        \shift_in_from_left[20][2] , \shift_in_from_left[22][1] , 
        \shift_in_from_left[23][0] }), .rs2_rdata(dmem_wdata[24]), .cmp_eq_in(
        b_cmp_eq_out[25]), .cmp_lt_in(b_cmp_lt_out[25]), .cmp_eq_out(
        b_cmp_eq_out[24]), .cmp_lt_out(b_cmp_lt_out[24]) );
  bitslice_7 \bitslices[25].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[25]), .alu_out(dmem_addr[25]), .alu_cin(
        b_alu_cin[25]), .alu_cout(b_alu_cin[26]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[25]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[25]), 
        .pc_cout(b_pc_cin[26]), .pc(imem_addr[25]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[25][4] , \shift_in_from_left[9][3] , 
        \shift_in_from_left[17][2] , \shift_in_from_left[21][1] , 
        \shift_in_from_left[23][0] }), .shift_in_from_left({shift_msb, 
        shift_msb, \shift_in_from_left[25][2] , \shift_in_from_left[25][1] , 
        \shift_in_from_left[25][0] }), .shift_out({SYNOPSYS_UNCONNECTED__25, 
        \shift_in_from_left[9][4] , \shift_in_from_left[17][3] , 
        \shift_in_from_left[21][2] , \shift_in_from_left[23][1] , 
        \shift_in_from_left[24][0] }), .rs2_rdata(dmem_wdata[25]), .cmp_eq_in(
        b_cmp_eq_out[26]), .cmp_lt_in(b_cmp_lt_out[26]), .cmp_eq_out(
        b_cmp_eq_out[25]), .cmp_lt_out(b_cmp_lt_out[25]) );
  bitslice_6 \bitslices[26].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[26]), .alu_out(dmem_addr[26]), .alu_cin(
        b_alu_cin[26]), .alu_cout(b_alu_cin[27]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[26]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[26]), 
        .pc_cout(b_pc_cin[27]), .pc(imem_addr[26]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[26][4] , \shift_in_from_left[10][3] , 
        \shift_in_from_left[18][2] , \shift_in_from_left[22][1] , 
        \shift_in_from_left[24][0] }), .shift_in_from_left({shift_msb, 
        shift_msb, \shift_in_from_left[26][2] , \shift_in_from_left[26][1] , 
        \shift_in_from_left[26][0] }), .shift_out({SYNOPSYS_UNCONNECTED__26, 
        \shift_in_from_left[10][4] , \shift_in_from_left[18][3] , 
        \shift_in_from_left[22][2] , \shift_in_from_left[24][1] , 
        \shift_in_from_left[25][0] }), .rs2_rdata(dmem_wdata[26]), .cmp_eq_in(
        b_cmp_eq_out[27]), .cmp_lt_in(b_cmp_lt_out[27]), .cmp_eq_out(
        b_cmp_eq_out[26]), .cmp_lt_out(b_cmp_lt_out[26]) );
  bitslice_5 \bitslices[27].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[27]), .alu_out(dmem_addr[27]), .alu_cin(
        b_alu_cin[27]), .alu_cout(b_alu_cin[28]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[27]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[27]), 
        .pc_cout(b_pc_cin[28]), .pc(imem_addr[27]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[27][4] , \shift_in_from_left[11][3] , 
        \shift_in_from_left[19][2] , \shift_in_from_left[23][1] , 
        \shift_in_from_left[25][0] }), .shift_in_from_left({shift_msb, 
        shift_msb, \shift_in_from_left[27][2] , \shift_in_from_left[27][1] , 
        \shift_in_from_left[27][0] }), .shift_out({SYNOPSYS_UNCONNECTED__27, 
        \shift_in_from_left[11][4] , \shift_in_from_left[19][3] , 
        \shift_in_from_left[23][2] , \shift_in_from_left[25][1] , 
        \shift_in_from_left[26][0] }), .rs2_rdata(dmem_wdata[27]), .cmp_eq_in(
        b_cmp_eq_out[28]), .cmp_lt_in(b_cmp_lt_out[28]), .cmp_eq_out(
        b_cmp_eq_out[27]), .cmp_lt_out(b_cmp_lt_out[27]) );
  bitslice_4 \bitslices[28].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[28]), .alu_out(dmem_addr[28]), .alu_cin(
        b_alu_cin[28]), .alu_cout(b_alu_cin[29]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[28]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b0), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[28]), 
        .pc_cout(b_pc_cin[29]), .pc(imem_addr[28]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[28][4] , \shift_in_from_left[12][3] , 
        \shift_in_from_left[20][2] , \shift_in_from_left[24][1] , 
        \shift_in_from_left[26][0] }), .shift_in_from_left({shift_msb, 
        shift_msb, shift_msb, \shift_in_from_left[28][1] , 
        \shift_in_from_left[28][0] }), .shift_out({SYNOPSYS_UNCONNECTED__28, 
        \shift_in_from_left[12][4] , \shift_in_from_left[20][3] , 
        \shift_in_from_left[24][2] , \shift_in_from_left[26][1] , 
        \shift_in_from_left[27][0] }), .rs2_rdata(dmem_wdata[28]), .cmp_eq_in(
        b_cmp_eq_out[29]), .cmp_lt_in(b_cmp_lt_out[29]), .cmp_eq_out(
        b_cmp_eq_out[28]), .cmp_lt_out(b_cmp_lt_out[28]) );
  bitslice_3 \bitslices[29].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[29]), .alu_out(dmem_addr[29]), .alu_cin(
        b_alu_cin[29]), .alu_cout(b_alu_cin[30]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[29]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b1), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[29]), 
        .pc_cout(b_pc_cin[30]), .pc(imem_addr[29]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[29][4] , \shift_in_from_left[13][3] , 
        \shift_in_from_left[21][2] , \shift_in_from_left[25][1] , 
        \shift_in_from_left[27][0] }), .shift_in_from_left({shift_msb, 
        shift_msb, shift_msb, \shift_in_from_left[29][1] , 
        \shift_in_from_left[29][0] }), .shift_out({SYNOPSYS_UNCONNECTED__29, 
        \shift_in_from_left[13][4] , \shift_in_from_left[21][3] , 
        \shift_in_from_left[25][2] , \shift_in_from_left[27][1] , 
        \shift_in_from_left[28][0] }), .rs2_rdata(dmem_wdata[29]), .cmp_eq_in(
        b_cmp_eq_out[30]), .cmp_lt_in(b_cmp_lt_out[30]), .cmp_eq_out(
        b_cmp_eq_out[29]), .cmp_lt_out(b_cmp_lt_out[29]) );
  bitslice_2 \bitslices[30].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[30]), .alu_out(dmem_addr[30]), .alu_cin(
        b_alu_cin[30]), .alu_cout(b_alu_cin[31]), .lb(dmem_rdata[7]), .lh(
        dmem_rdata[15]), .lw(dmem_rdata[30]), .lbu(1'b0), .lhu(1'b0), 
        .pc_reset_value(1'b1), .pc_adder_4(1'b0), .pc_cin(b_pc_cin[30]), 
        .pc_cout(b_pc_cin[31]), .pc(imem_addr[30]), .shift_amount(
        alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[30][4] , \shift_in_from_left[14][3] , 
        \shift_in_from_left[22][2] , \shift_in_from_left[26][1] , 
        \shift_in_from_left[28][0] }), .shift_in_from_left({shift_msb, 
        shift_msb, shift_msb, shift_msb, \shift_in_from_left[30][0] }), 
        .shift_out({SYNOPSYS_UNCONNECTED__30, \shift_in_from_left[14][4] , 
        \shift_in_from_left[22][3] , \shift_in_from_left[26][2] , 
        \shift_in_from_left[28][1] , \shift_in_from_left[29][0] }), 
        .rs2_rdata(dmem_wdata[30]), .cmp_eq_in(b_cmp_eq_out[31]), .cmp_lt_in(
        b_cmp_lt_out[31]), .cmp_eq_out(b_cmp_eq_out[30]), .cmp_lt_out(
        b_cmp_lt_out[30]) );
  bitslice_1 \bitslices[31].bitslice  ( .clk(n4), .rst(rst), .rs1_sel(rs1_sel), 
        .rs2_sel(rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_op(
        alu_op), .shift_dir(n1), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(
        pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(1'b0), .imm(imm[31]), .alu_out(dmem_addr[31]), .alu_cin(
        b_alu_cin[31]), .lb(dmem_rdata[7]), .lh(dmem_rdata[15]), .lw(
        dmem_rdata[31]), .lbu(1'b0), .lhu(1'b0), .pc_reset_value(1'b0), 
        .pc_adder_4(1'b0), .pc_cin(b_pc_cin[31]), .pc(imem_addr[31]), 
        .shift_amount(alu_mux_2_out[4:0]), .shift_in_from_right({
        \shift_in_from_right[31][4] , \shift_in_from_left[15][3] , 
        \shift_in_from_left[23][2] , \shift_in_from_left[27][1] , 
        \shift_in_from_left[29][0] }), .shift_in_from_left({shift_msb, 
        shift_msb, shift_msb, shift_msb, shift_msb}), .shift_out({
        SYNOPSYS_UNCONNECTED__31, \shift_in_from_left[15][4] , 
        \shift_in_from_left[23][3] , \shift_in_from_left[27][2] , 
        \shift_in_from_left[29][1] , \shift_in_from_left[30][0] }), 
        .rs2_rdata(dmem_wdata[31]), .cmp_eq_in(1'b1), .cmp_lt_in(1'b0), 
        .cmp_eq_out(b_cmp_eq_out[31]), .cmp_lt_out(b_cmp_lt_out[31]), 
        .cmp_src_a(cmp_a_31), .cmp_src_b(cmp_b_31) );
  \buf  U3 ( .A(shift_dir), .Y(n1) );
  \buf  U4 ( .A(shift_dir), .Y(n2) );
  \buf  U5 ( .A(shift_dir), .Y(n3) );
  \buf  U6 ( .A(clk), .Y(n4) );
  \buf  U7 ( .A(clk), .Y(n5) );
  \buf  U8 ( .A(clk), .Y(n6) );
endmodule


module cpu ( clk, rst, imem_addr, imem_rdata, dmem_addr, dmem_write, 
        dmem_wmask, dmem_rdata, dmem_wdata );
  output [31:0] imem_addr;
  input [31:0] imem_rdata;
  output [31:0] dmem_addr;
  output [3:0] dmem_wmask;
  input [31:0] dmem_rdata;
  output [31:0] dmem_wdata;
  input clk, rst;
  output dmem_write;
  wire   cmp_b_31, cmp_a_31, cmp_eq, cmp_lt, cmp_out, pc_mux_sel, cmp_mux_sel,
         shift_dir, shift_msb, alu_cin, alu_inv_rs2, alu_mux_2_sel,
         alu_mux_1_sel;
  wire   [31:0] imm;
  wire   [2:0] rd_mux_sel;
  wire   [2:0] mem_mux_sel;
  wire   [1:0] alu_op;
  wire   [31:0] rd_sel;
  wire   [31:0] rs2_sel;
  wire   [31:0] rs1_sel;

  control control ( .clk(clk), .imem_rdata(imem_rdata), .dmem_write(dmem_write), .dmem_wmask(dmem_wmask), .rs1_sel(rs1_sel), .rs2_sel(rs2_sel), .rd_sel(
        rd_sel), .alu_mux_1_sel(alu_mux_1_sel), .alu_mux_2_sel(alu_mux_2_sel), 
        .alu_inv_rs2(alu_inv_rs2), .alu_cin(alu_cin), .alu_op(alu_op), 
        .shift_msb(shift_msb), .shift_dir(shift_dir), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), 
        .cmp_out(cmp_out), .imm(imm), .cmp_lt(cmp_lt), .cmp_eq(cmp_eq), 
        .cmp_a_31(cmp_a_31), .cmp_b_31(cmp_b_31) );
  datapath datapath ( .clk(clk), .rst(rst), .rs1_sel(rs1_sel), .rs2_sel(
        rs2_sel), .rd_sel(rd_sel), .alu_mux_1_sel(alu_mux_1_sel), 
        .alu_mux_2_sel(alu_mux_2_sel), .alu_inv_rs2(alu_inv_rs2), .alu_cin(
        alu_cin), .alu_op(alu_op), .shift_msb(shift_msb), .shift_dir(shift_dir), .cmp_mux_sel(cmp_mux_sel), .pc_mux_sel(pc_mux_sel), .mem_mux_sel(mem_mux_sel), .rd_mux_sel(rd_mux_sel), .cmp_out(cmp_out), .imm(imm), .cmp_lt(cmp_lt), 
        .cmp_eq(cmp_eq), .cmp_a_31(cmp_a_31), .cmp_b_31(cmp_b_31), .imem_addr(
        imem_addr), .dmem_addr(dmem_addr), .dmem_rdata(dmem_rdata), 
        .dmem_wdata(dmem_wdata) );
endmodule

