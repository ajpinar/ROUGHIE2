<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="26" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="con-molex">
<description>&lt;b&gt;Molex Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="87758-1216">
<description>&lt;b&gt;12 Pin - 2mm Dual Row Single Wafer, Vertical T/H HDR&lt;/b&gt;&lt;p&gt;
Source: http://www.molex.com/pdm_docs/sd/877580616_sd.pdf</description>
<wire x1="-5.85" y1="-1.9" x2="5.85" y2="-1.9" width="0.2032" layer="21"/>
<wire x1="5.85" y1="-1.9" x2="5.85" y2="-0.4" width="0.2032" layer="21"/>
<wire x1="5.85" y1="0.4" x2="5.85" y2="1.9" width="0.2032" layer="21"/>
<wire x1="5.85" y1="1.9" x2="-5.85" y2="1.9" width="0.2032" layer="21"/>
<wire x1="-5.85" y1="1.9" x2="-5.85" y2="0.4" width="0.2032" layer="21"/>
<wire x1="-5.85" y1="-0.4" x2="-5.85" y2="-1.9" width="0.2032" layer="21"/>
<wire x1="-5.85" y1="0.4" x2="-5.85" y2="-0.4" width="0.2032" layer="21" curve="-129.184564"/>
<wire x1="5.85" y1="-0.4" x2="5.85" y2="0.4" width="0.2032" layer="21" curve="-129.184564"/>
<pad name="1" x="-5" y="-1" drill="0.9" diameter="1.27"/>
<pad name="2" x="-5" y="1" drill="0.9" diameter="1.27"/>
<pad name="3" x="-3" y="-1" drill="0.9" diameter="1.27"/>
<pad name="4" x="-3" y="1" drill="0.9" diameter="1.27"/>
<pad name="5" x="-1" y="-1" drill="0.9" diameter="1.27"/>
<pad name="6" x="-1" y="1" drill="0.9" diameter="1.27"/>
<pad name="7" x="1" y="-1" drill="0.9" diameter="1.27"/>
<pad name="8" x="1" y="1" drill="0.9" diameter="1.27"/>
<pad name="9" x="3" y="-1" drill="0.9" diameter="1.27"/>
<pad name="10" x="3" y="1" drill="0.9" diameter="1.27"/>
<pad name="11" x="5" y="-1" drill="0.9" diameter="1.27"/>
<pad name="12" x="5" y="1" drill="0.9" diameter="1.27"/>
<text x="-5.65" y="-1.75" size="0.3048" layer="21" font="vector">1</text>
<text x="-5.62" y="-3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="0.73" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-5.25" y1="-1.25" x2="-4.75" y2="-0.75" layer="51"/>
<rectangle x1="-5.25" y1="0.75" x2="-4.75" y2="1.25" layer="51"/>
<rectangle x1="-3.25" y1="-1.25" x2="-2.75" y2="-0.75" layer="51"/>
<rectangle x1="-3.25" y1="0.75" x2="-2.75" y2="1.25" layer="51"/>
<rectangle x1="-1.25" y1="-1.25" x2="-0.75" y2="-0.75" layer="51"/>
<rectangle x1="-1.25" y1="0.75" x2="-0.75" y2="1.25" layer="51"/>
<rectangle x1="0.75" y1="-1.25" x2="1.25" y2="-0.75" layer="51"/>
<rectangle x1="0.75" y1="0.75" x2="1.25" y2="1.25" layer="51"/>
<rectangle x1="2.75" y1="-1.25" x2="3.25" y2="-0.75" layer="51"/>
<rectangle x1="2.75" y1="0.75" x2="3.25" y2="1.25" layer="51"/>
<rectangle x1="4.75" y1="-1.25" x2="5.25" y2="-0.75" layer="51"/>
<rectangle x1="4.75" y1="0.75" x2="5.25" y2="1.25" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="MV">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<text x="-0.762" y="1.397" size="1.778" layer="96">&gt;VALUE</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="M">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="87758-1216" prefix="X">
<description>&lt;b&gt;12 Pin - 2mm Dual Row Single Wafer, Vertical T/H HDR&lt;/b&gt;&lt;p&gt;
Source: http://www.molex.com/pdm_docs/sd/877580616_sd.pdf</description>
<gates>
<gate name="-1" symbol="MV" x="-10.16" y="7.62" addlevel="always"/>
<gate name="-2" symbol="MV" x="10.16" y="7.62" addlevel="always"/>
<gate name="-3" symbol="M" x="-10.16" y="5.08" addlevel="always"/>
<gate name="-4" symbol="M" x="10.16" y="5.08" addlevel="always"/>
<gate name="-5" symbol="M" x="-10.16" y="2.54" addlevel="always"/>
<gate name="-6" symbol="M" x="10.16" y="2.54" addlevel="always"/>
<gate name="-7" symbol="M" x="-10.16" y="0" addlevel="always"/>
<gate name="-8" symbol="M" x="10.16" y="0" addlevel="always"/>
<gate name="-9" symbol="M" x="-10.16" y="-2.54" addlevel="always"/>
<gate name="-10" symbol="M" x="10.16" y="-2.54" addlevel="always"/>
<gate name="-11" symbol="M" x="-10.16" y="-5.08" addlevel="always"/>
<gate name="-12" symbol="M" x="10.16" y="-5.08" addlevel="always"/>
</gates>
<devices>
<device name="" package="87758-1216">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-10" pin="S" pad="10"/>
<connect gate="-11" pin="S" pad="11"/>
<connect gate="-12" pin="S" pad="12"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
<connect gate="-9" pin="S" pad="9"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="87758-1216" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="25M5671" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="relay">
<description>&lt;b&gt;Relays&lt;/b&gt;&lt;p&gt;
&lt;ul&gt;
&lt;li&gt;Eichhoff
&lt;li&gt;Finder
&lt;li&gt;Fujitsu
&lt;li&gt;HAMLIN
&lt;li&gt;OMRON
&lt;li&gt;Matsushita
&lt;li&gt;NAiS
&lt;li&gt;Siemens
&lt;li&gt;Schrack
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="RTE">
<pad name="P$1" x="0" y="0" drill="1.3"/>
<pad name="P$2" x="15.2908" y="0" drill="1.3"/>
<pad name="P$3" x="20.32" y="0" drill="1.3"/>
<pad name="P$4" x="25.3492" y="0" drill="1.3"/>
<pad name="P$5" x="25.3492" y="7.493" drill="1.3"/>
<pad name="P$6" x="20.32" y="7.493" drill="1.3"/>
<pad name="P$7" x="15.2908" y="7.493" drill="1.3"/>
<pad name="P$8" x="0" y="7.493" drill="1.3"/>
<wire x1="-2.3368" y1="10.0965" x2="26.67" y2="10.0965" width="0.127" layer="21"/>
<wire x1="26.67" y1="10.0965" x2="26.67" y2="-2.6035" width="0.127" layer="21"/>
<wire x1="26.67" y1="-2.6035" x2="-2.3368" y2="-2.6035" width="0.127" layer="21"/>
<wire x1="-2.3368" y1="-2.6035" x2="-2.3368" y2="10.0965" width="0.127" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="K+-">
<wire x1="-3.81" y1="-1.905" x2="-1.905" y2="-1.905" width="0.254" layer="94"/>
<wire x1="3.81" y1="-1.905" x2="3.81" y2="1.905" width="0.254" layer="94"/>
<wire x1="3.81" y1="1.905" x2="1.905" y2="1.905" width="0.254" layer="94"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="0" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="0" y1="-1.905" x2="3.81" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="2.54" x2="0" y2="1.905" width="0.1524" layer="94"/>
<wire x1="0" y1="1.905" x2="-3.81" y2="1.905" width="0.254" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="1.905" y2="1.905" width="0.1524" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="0" y2="-1.905" width="0.254" layer="94"/>
<wire x1="1.905" y1="1.905" x2="0" y2="1.905" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.667" x2="-0.508" y2="2.667" width="0.1524" layer="94"/>
<wire x1="-0.762" y1="2.921" x2="-0.762" y2="2.413" width="0.1524" layer="94"/>
<wire x1="-1.016" y1="-2.667" x2="-0.508" y2="-2.667" width="0.1524" layer="94"/>
<text x="1.27" y="2.921" size="1.778" layer="96">&gt;VALUE</text>
<text x="1.27" y="5.08" size="1.778" layer="95">&gt;PART</text>
<pin name="-" x="0" y="-5.08" visible="pad" length="short" direction="pas" rot="R90"/>
<pin name="+" x="0" y="5.08" visible="pad" length="short" direction="pas" rot="R270"/>
</symbol>
<symbol name="U">
<wire x1="3.175" y1="5.08" x2="1.905" y2="5.08" width="0.254" layer="94"/>
<wire x1="-3.175" y1="5.08" x2="-1.905" y2="5.08" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="2.54" y2="5.715" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="0" y2="0" width="0.254" layer="94"/>
<circle x="0" y="1.27" radius="0.127" width="0.4064" layer="94"/>
<text x="2.54" y="0" size="1.778" layer="95">&gt;PART</text>
<pin name="O" x="5.08" y="5.08" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="S" x="-5.08" y="5.08" visible="pad" length="short" direction="pas"/>
<pin name="P" x="0" y="-2.54" visible="pad" length="short" direction="pas" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="RTE">
<gates>
<gate name="G$1" symbol="K+-" x="-22.86" y="10.16"/>
<gate name="G$2" symbol="U" x="-5.08" y="22.86"/>
<gate name="G$3" symbol="U" x="-5.08" y="-5.08"/>
</gates>
<devices>
<device name="" package="RTE">
<connects>
<connect gate="G$1" pin="+" pad="P$1"/>
<connect gate="G$1" pin="-" pad="P$8"/>
<connect gate="G$2" pin="O" pad="P$2"/>
<connect gate="G$2" pin="P" pad="P$3"/>
<connect gate="G$2" pin="S" pad="P$4"/>
<connect gate="G$3" pin="O" pad="P$7"/>
<connect gate="G$3" pin="P" pad="P$6"/>
<connect gate="G$3" pin="S" pad="P$5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="+24V">
<wire x1="1.27" y1="-0.635" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-0.635" width="0.254" layer="94"/>
<wire x1="1.27" y1="-0.635" x2="0" y2="1.27" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="-1.27" y2="-0.635" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+24V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="+24V" prefix="P+">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+24V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="GND" prefix="GND">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="testpad">
<description>&lt;b&gt;Test Pins/Pads&lt;/b&gt;&lt;p&gt;
Cream on SMD OFF.&lt;br&gt;
new: Attribute TP_SIGNAL_NAME&lt;br&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="P2-38-17">
<description>&lt;b&gt;TEST PAD&lt;/b&gt;</description>
<circle x="-1.905" y="0" radius="0.8128" width="0.1524" layer="51"/>
<circle x="1.905" y="0" radius="0.8128" width="0.1524" layer="51"/>
<pad name="TP-1" x="-1.905" y="0" drill="1.7018" diameter="2.1208" shape="long" rot="R90"/>
<pad name="TP-2" x="1.905" y="0" drill="1.7018" diameter="2.1208" shape="long" rot="R90"/>
<text x="-2.54" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-2.54" y="-5.715" size="1" layer="37">&gt;TP_SIGNAL_NAME</text>
<rectangle x1="-2.2352" y1="-0.3302" x2="-1.5748" y2="0.3302" layer="51"/>
<rectangle x1="1.5748" y1="-0.3302" x2="2.2352" y2="0.3302" layer="51"/>
</package>
<package name="P2-38-20">
<description>&lt;b&gt;TEST PAD&lt;/b&gt;</description>
<circle x="-1.905" y="0" radius="1.016" width="0.1524" layer="51"/>
<circle x="1.905" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="TP-1" x="-1.905" y="0" drill="2.0066" diameter="2.54" shape="long" rot="R90"/>
<pad name="TP-2" x="1.905" y="0" drill="2.0066" diameter="2.54" shape="long" rot="R90"/>
<text x="-2.54" y="2.794" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-2.54" y="-6.35" size="1" layer="37">&gt;TP_SIGNAL_NAME</text>
<rectangle x1="-2.2352" y1="-0.3302" x2="-1.5748" y2="0.3302" layer="51"/>
<rectangle x1="1.5748" y1="-0.3302" x2="2.2352" y2="0.3302" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="TP">
<wire x1="-0.762" y1="-0.762" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="0.762" y2="-0.762" width="0.254" layer="94"/>
<wire x1="0.762" y1="-0.762" x2="0" y2="-1.524" width="0.254" layer="94"/>
<wire x1="0" y1="-1.524" x2="-0.762" y2="-0.762" width="0.254" layer="94"/>
<text x="-1.27" y="1.27" size="1.778" layer="95">&gt;NAME</text>
<text x="1.27" y="-1.27" size="1.778" layer="97">&gt;TP_SIGNAL_NAME</text>
<pin name="TP" x="0" y="-2.54" visible="off" length="short" direction="in" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="TP2" prefix="TP">
<description>&lt;b&gt;Test pad&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="TP" x="0" y="0" addlevel="always"/>
<gate name="B" symbol="TP" x="7.62" y="0" addlevel="always"/>
</gates>
<devices>
<device name="P2-38-17" package="P2-38-17">
<connects>
<connect gate="A" pin="TP" pad="TP-1"/>
<connect gate="B" pin="TP" pad="TP-2"/>
</connects>
<technologies>
<technology name="">
<attribute name="TP_SIGNAL_NAME" value="" constant="no"/>
</technology>
</technologies>
</device>
<device name="P2-38-20" package="P2-38-20">
<connects>
<connect gate="A" pin="TP" pad="TP-1"/>
<connect gate="B" pin="TP" pad="TP-2"/>
</connects>
<technologies>
<technology name="">
<attribute name="TP_SIGNAL_NAME" value="" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="diode">
<description>&lt;b&gt;Diodes&lt;/b&gt;&lt;p&gt;
Based on the following sources:
&lt;ul&gt;
&lt;li&gt;Motorola : www.onsemi.com
&lt;li&gt;Fairchild : www.fairchildsemi.com
&lt;li&gt;Philips : www.semiconductors.com
&lt;li&gt;Vishay : www.vishay.de
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="CD1206">
<smd name="A" x="-1.4986" y="0" dx="1.397" dy="1.6002" layer="1"/>
<smd name="K" x="1.4986" y="0" dx="1.397" dy="1.6002" layer="1"/>
</package>
</packages>
<symbols>
<symbol name="D">
<wire x1="-1.27" y1="-1.27" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="-1.27" y2="1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="-1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="1.27" y2="-1.27" width="0.254" layer="94"/>
<text x="2.54" y="0.4826" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-2.3114" size="1.778" layer="96">&gt;VALUE</text>
<pin name="A" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
<pin name="C" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="CD1206-S01575">
<gates>
<gate name="G$1" symbol="D" x="0" y="0"/>
</gates>
<devices>
<device name="" package="CD1206">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="relay" deviceset="RTE" device=""/>
<part name="P+1" library="supply1" deviceset="+24V" device=""/>
<part name="P+2" library="supply1" deviceset="+24V" device=""/>
<part name="GND1" library="supply1" deviceset="GND" device=""/>
<part name="TP4" library="testpad" deviceset="TP2" device="P2-38-20"/>
<part name="TP5" library="testpad" deviceset="TP2" device="P2-38-20"/>
<part name="X2" library="con-molex" deviceset="87758-1216" device=""/>
<part name="U$2" library="diode" deviceset="CD1206-S01575" device=""/>
</parts>
<sheets>
<sheet>
<plain>
<text x="172.72" y="45.72" size="1.778" layer="91">Pump first phase</text>
<text x="172.72" y="17.78" size="1.778" layer="91">Pump third phase</text>
<text x="38.1" y="73.66" size="1.778" layer="91">Power in</text>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="101.6" y="40.64"/>
<instance part="U$1" gate="G$2" x="134.62" y="48.26" rot="R270"/>
<instance part="U$1" gate="G$3" x="134.62" y="20.32" rot="R270"/>
<instance part="P+1" gate="1" x="10.16" y="104.14"/>
<instance part="P+2" gate="1" x="101.6" y="63.5"/>
<instance part="GND1" gate="1" x="10.16" y="27.94"/>
<instance part="TP4" gate="A" x="121.92" y="48.26" rot="R90"/>
<instance part="TP4" gate="B" x="121.92" y="20.32" rot="R90"/>
<instance part="TP5" gate="A" x="160.02" y="50.8" rot="R270"/>
<instance part="TP5" gate="B" x="160.02" y="20.32" rot="R270"/>
<instance part="X2" gate="-1" x="27.94" y="76.2"/>
<instance part="X2" gate="-2" x="27.94" y="73.66"/>
<instance part="X2" gate="-3" x="27.94" y="86.36"/>
<instance part="X2" gate="-4" x="25.4" y="53.34"/>
<instance part="X2" gate="-5" x="27.94" y="88.9"/>
<instance part="X2" gate="-6" x="25.4" y="50.8"/>
<instance part="X2" gate="-7" x="27.94" y="91.44"/>
<instance part="X2" gate="-8" x="25.4" y="48.26"/>
<instance part="X2" gate="-9" x="53.34" y="99.06"/>
<instance part="X2" gate="-10" x="53.34" y="96.52"/>
<instance part="X2" gate="-11" x="76.2" y="30.48" rot="R180"/>
<instance part="X2" gate="-12" x="53.34" y="116.84"/>
<instance part="U$2" gate="G$1" x="86.36" y="40.64" rot="R90"/>
</instances>
<busses>
</busses>
<nets>
<net name="+24V" class="0">
<segment>
<pinref part="P+1" gate="1" pin="+24V"/>
<wire x1="25.4" y1="76.2" x2="10.16" y2="76.2" width="0.1524" layer="91"/>
<wire x1="10.16" y1="76.2" x2="10.16" y2="86.36" width="0.1524" layer="91"/>
<pinref part="X2" gate="-1" pin="S"/>
<pinref part="X2" gate="-7" pin="S"/>
<wire x1="10.16" y1="86.36" x2="10.16" y2="88.9" width="0.1524" layer="91"/>
<wire x1="10.16" y1="88.9" x2="10.16" y2="91.44" width="0.1524" layer="91"/>
<wire x1="10.16" y1="91.44" x2="10.16" y2="101.6" width="0.1524" layer="91"/>
<wire x1="25.4" y1="91.44" x2="10.16" y2="91.44" width="0.1524" layer="91"/>
<junction x="10.16" y="91.44"/>
<pinref part="X2" gate="-5" pin="S"/>
<wire x1="25.4" y1="88.9" x2="10.16" y2="88.9" width="0.1524" layer="91"/>
<junction x="10.16" y="88.9"/>
<pinref part="X2" gate="-3" pin="S"/>
<wire x1="25.4" y1="86.36" x2="10.16" y2="86.36" width="0.1524" layer="91"/>
<junction x="10.16" y="86.36"/>
</segment>
<segment>
<pinref part="U$1" gate="G$1" pin="+"/>
<pinref part="P+2" gate="1" pin="+24V"/>
<wire x1="101.6" y1="60.96" x2="101.6" y2="53.34" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="C"/>
<wire x1="101.6" y1="53.34" x2="101.6" y2="45.72" width="0.1524" layer="91"/>
<wire x1="101.6" y1="53.34" x2="86.36" y2="53.34" width="0.1524" layer="91"/>
<wire x1="86.36" y1="53.34" x2="86.36" y2="43.18" width="0.1524" layer="91"/>
<junction x="101.6" y="53.34"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="10.16" y1="73.66" x2="10.16" y2="53.34" width="0.1524" layer="91"/>
<pinref part="X2" gate="-2" pin="S"/>
<wire x1="10.16" y1="53.34" x2="10.16" y2="50.8" width="0.1524" layer="91"/>
<wire x1="10.16" y1="50.8" x2="10.16" y2="48.26" width="0.1524" layer="91"/>
<wire x1="10.16" y1="48.26" x2="10.16" y2="30.48" width="0.1524" layer="91"/>
<wire x1="25.4" y1="73.66" x2="10.16" y2="73.66" width="0.1524" layer="91"/>
<pinref part="X2" gate="-4" pin="S"/>
<wire x1="22.86" y1="53.34" x2="10.16" y2="53.34" width="0.1524" layer="91"/>
<junction x="10.16" y="53.34"/>
<pinref part="X2" gate="-6" pin="S"/>
<wire x1="22.86" y1="50.8" x2="10.16" y2="50.8" width="0.1524" layer="91"/>
<junction x="10.16" y="50.8"/>
<pinref part="X2" gate="-8" pin="S"/>
<wire x1="22.86" y1="48.26" x2="10.16" y2="48.26" width="0.1524" layer="91"/>
<junction x="10.16" y="48.26"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="U$1" gate="G$2" pin="O"/>
<wire x1="139.7" y1="43.18" x2="139.7" y2="40.64" width="0.1524" layer="91"/>
<wire x1="139.7" y1="40.64" x2="149.86" y2="40.64" width="0.1524" layer="91"/>
<wire x1="149.86" y1="40.64" x2="149.86" y2="50.8" width="0.1524" layer="91"/>
<pinref part="TP5" gate="A" pin="TP"/>
<wire x1="149.86" y1="50.8" x2="157.48" y2="50.8" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$3" pin="S"/>
<wire x1="139.7" y1="25.4" x2="139.7" y2="40.64" width="0.1524" layer="91"/>
<junction x="139.7" y="40.64"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="U$1" gate="G$3" pin="O"/>
<wire x1="139.7" y1="15.24" x2="139.7" y2="10.16" width="0.1524" layer="91"/>
<wire x1="139.7" y1="10.16" x2="144.78" y2="10.16" width="0.1524" layer="91"/>
<wire x1="144.78" y1="10.16" x2="149.86" y2="10.16" width="0.1524" layer="91"/>
<wire x1="149.86" y1="10.16" x2="149.86" y2="20.32" width="0.1524" layer="91"/>
<pinref part="TP5" gate="B" pin="TP"/>
<wire x1="149.86" y1="20.32" x2="157.48" y2="20.32" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$2" pin="S"/>
<wire x1="139.7" y1="53.34" x2="139.7" y2="58.42" width="0.1524" layer="91"/>
<wire x1="139.7" y1="58.42" x2="144.78" y2="58.42" width="0.1524" layer="91"/>
<wire x1="144.78" y1="58.42" x2="144.78" y2="10.16" width="0.1524" layer="91"/>
<junction x="144.78" y="10.16"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="U$1" gate="G$2" pin="P"/>
<pinref part="TP4" gate="A" pin="TP"/>
<wire x1="132.08" y1="48.26" x2="124.46" y2="48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="U$1" gate="G$3" pin="P"/>
<pinref part="TP4" gate="B" pin="TP"/>
<wire x1="132.08" y1="20.32" x2="124.46" y2="20.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="-"/>
<wire x1="101.6" y1="35.56" x2="101.6" y2="30.48" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="A"/>
<wire x1="86.36" y1="30.48" x2="101.6" y2="30.48" width="0.1524" layer="91"/>
<wire x1="86.36" y1="38.1" x2="86.36" y2="30.48" width="0.1524" layer="91"/>
<pinref part="X2" gate="-11" pin="S"/>
<wire x1="78.74" y1="30.48" x2="86.36" y2="30.48" width="0.1524" layer="91"/>
<junction x="86.36" y="30.48"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
