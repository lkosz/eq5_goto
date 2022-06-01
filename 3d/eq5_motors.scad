$fn=100;

module nakretka(){

hull(){
translate([0,-1,0])cylinder(d=15, h=10);
translate([0,1,0])cylinder(d=15, h=10);
}
hull(){
translate([0,-1,0])cylinder(d=6, h=44, center=true);
translate([0,1,0])cylinder(d=6, h=44, center=true);
}


}

module nakretka2(){

hull(){
translate([-4.1,-2-4.1,0])cube([8.2,8.2,10]);
translate([-4.1,2-4.1,0])cube([8.2,8.2,10]);
}
hull(){
translate([0,-1,0])cylinder(d=5, h=44, center=true);
translate([0,1,0])cylinder(d=5, h=44, center=true);
}


}

module rounded_cube(tab,d=3){
x=tab[0];
y=tab[1];
z=tab[2];

hull(){
translate([d/2,d/2,d/2])sphere(d=d);
translate([x-d/2,d/2,d/2])sphere(d=d);
translate([x-d/2,y-d/2,d/2])sphere(d=d);
translate([d/2,y-d/2,d/2])sphere(d=d);
translate([d/2,d/2,z-d/2])sphere(d=d);
translate([x-d/2,d/2,z-d/2])sphere(d=d);
translate([x-d/2,y-d/2,z-d/2])sphere(d=d);
translate([d/2,y-d/2,z-d/2])sphere(d=d);
}
}


module nema17_helper(slide=8){
  
  union(){
 hull(){
 translate([0,slide/2,0]) cylinder(d=22, h=20, center=true);
  translate([0,-1*slide/2,0]) cylinder(d=22, h=20, center=true);

   }

hull(){
translate([0,slide/2,0]) translate([15.5,15.5]) cylinder(d=3.5, h=20, center=true);
translate([0,-1*slide/2,0])translate([15.5,15.5]) cylinder(d=3.5, h=20, center=true);
}
hull(){
translate([0,slide/2,0]) translate([-15.5,15.5]) cylinder(d=3.5, h=20, center=true);
translate([0,-1*slide/2,0])translate([-15.5,15.5]) cylinder(d=3.5, h=20, center=true);
}
hull(){
translate([0,slide/2,0]) translate([15.5,-15.5]) cylinder(d=3.5, h=20, center=true);
translate([0,-1*slide/2,0])translate([15.5,-15.5]) cylinder(d=3.5, h=20, center=true);
}
hull(){
translate([0,slide/2,0]) translate([-15.5,-15.5]) cylinder(d=3.5, h=20, center=true);
translate([0,-1*slide/2,0])translate([-15.5,-15.5]) cylinder(d=3.5, h=20, center=true);
}   
   }
  
}





module RA(){

difference(){
union(){


translate([0,0,60])difference(){
union(){
rounded_cube([85.5-17+5-2,57+32,9],d=3);
translate([-12,11,0])rounded_cube([15,35,9]);
}
translate([-100,-100,-1])cube([1000,1000,5]);

}



rounded_cube([85.5-17+5-2,57,5+42+1],d=3);

translate([0,0,37.5-2])difference(){
rounded_cube([85.5-17+5-2,57,5+42+3],d=3);
translate([-1,-1,-1])cube([100,100,10]);
translate([-1,-1,13])cube([100,100,100]);
}

translate([30.5,8,-6])cube([20.5,57,15]);
translate([21,23+8,-6])cube([38,34,15]);

translate([-12,(57-35)/2+35,30.5-2])rotate([0,0,270])difference(){
union(){
hull(){
rounded_cube([35,15,20]);
translate([0,14,-10])cube([35,1,1]);
}
translate([0,0,13])difference(){
rounded_cube([35,15,20]);
translate([-1,-1,-1])cube([100,100,5]);
translate([-1,-1,7])cube([100,100,50]);
}
}
translate([4.5,3,18.1])cube([26,9,2]);
translate([(35-22)/2,3,-1])cube([22,8.5,100]);
}
}
translate([-0.01,57-7+0.01+7,0])cube([210,100,10]);
translate([40.5,38+7,-1])nakretka();

translate([3,2,5])cube([85-16-8,47+4,52]);
translate([76-10,4+21,5+21])rotate([0,90,0])nema17_helper(slide=8);


translate([85.5-17,3,42])cylinder(d=2, h=100);
translate([0,13.5,42])cylinder(d=2, h=100);
translate([0,13.5+30,42])cylinder(d=2, h=100);
translate([85.5-17,54.5,42])cylinder(d=2, h=100);
translate([-3,(57-35)/2+14,31])cube([10,5,20]);

//translate([-20,-1,5])cube([82,52,100]);
color("red")translate([-9,16.5,50])cube([8.5,24,100]);

translate([85.5-17,3,66])cylinder(d=5, h=100);
translate([0,13.5,66])cylinder(d=5, h=100);
translate([0,13.5+30,66])cylinder(d=5, h=100);
translate([85.5-17,54.5,66])cylinder(d=5, h=100);
translate([40+3,57+22+45-8-7,20])cylinder(d=90,h=100);

translate([23,2.5,50])cube([40.5,40.5,100]);
}
}






module DEC(){

difference(){
union(){




translate([0,0,100]){
difference(){
union(){
rounded_cube([70,60-8.5-1.5-2,9]);
translate([3,11-5,0])rotate([0,0,90])rounded_cube([35,15,9]);

}
translate([-100,-100,-1])cube([1000,1000,5]);
}

}





rounded_cube([70,60-8.5-1.5-2,55]);


translate([0,0,62-15]){
difference(){
rounded_cube([70,60-8.5-1.5-2,70]);
translate([-1,-1,-1])cube([100,100,5]);
translate([-1,-1,8])cube([100,100,500]);
}
}

translate([(72-51)/2+5,-17.99,0]){
translate([0,0,27-18])cube([10,18,18]);
translate([16,18-6.5,0])cube([19,6.5,27]);
translate([25+10+6,0,27-18])cube([10,18,18]);
hull(){
translate([0,0,27])cube([51,18,1]);
translate([0,17,37])cube([51,1,1]);
}
}


translate([-12,(57-35)/2+35-5,35])rotate([0,0,270])difference(){
union(){
hull(){
rounded_cube([35,15,20]);
translate([0,14,-10])cube([35,1,1]);
}
translate([0,0,13])difference(){
rounded_cube([35,15,20]);
translate([-1,-1,-1])cube([100,100,5]);
translate([-1,-1,7])cube([100,100,50]);
}
}
translate([4.5,3,18.1])cube([26,9,2]);
translate([(35-22)/2,3,-1])cube([22,8.5,100]);
}

}


translate([5,2,5])cube([70-10,60-10-8.5+2.5,55]);
translate([68,48/2,55/2+3])rotate([90,0,90])nema17_helper(slide=7);


translate([72/2+5,9,11])rotate([90,0,0])nakretka2();
translate([72/2+5,9,12])rotate([90,0,0])nakretka2();

translate([2.5,2.5+1,25])cylinder(d=2, h=100);
translate([70-2.5,2.5+1,25])cylinder(d=2, h=100);
translate([2.5,60-2.5-8.5-1.5-1-2,25])cylinder(d=2, h=100);
translate([70-2.5,60-2.5-8.5-1.5-1-2,25])cylinder(d=2, h=100);

translate([-5,13-5,45])cylinder(d=2, h=100);
translate([-5,44-5,45])cylinder(d=2, h=100);


translate([0,0,106]){
translate([2.5,3.5,0])cylinder(d=4, h=100);
translate([70-2.5,3.5,0])cylinder(d=4, h=100);
translate([2.5,60-2.5-8.5-1.5-1-2,0])cylinder(d=4, h=100);
translate([70-2.5,60-2.5-8.5-1.5-1-2,0])cylinder(d=4, h=100);
translate([-5,13-5,0])cylinder(d=4, h=100);
translate([-5,44-5,0])cylinder(d=4, h=100);
}
translate([0,0,100]){
translate([2.5,3.5,0])cylinder(d=2.5, h=100);
translate([70-2.5,3.5,0])cylinder(d=2.5, h=100);
translate([2.5,60-2.5-8.5-1.5-1-2,0])cylinder(d=2.5, h=100);
translate([70-2.5,60-2.5-8.5-1.5-1-2,0])cylinder(d=2.5, h=100);
translate([-5,13-5,0])cylinder(d=2.5, h=100);
translate([-5,44-5,0])cylinder(d=2.5, h=100);
}
translate([-9,16.5-5,60])cube([8.5,24,100]);
translate([-3,21,49-15])cube([10,5,40]);


}

}



RA();
translate([200,0,0])DEC();





