$fn=100;

module uchwyt1(){
cylinder(d=12, h=31);
}

module uchwyt2(){
cylinder(d=12, h=30);
}

module dziurka1(){
translate([0,0,-0.01])cylinder(d=5,h=35);
translate([0,0,-0.01])cylinder(d1=10,d2=5,h=4.5);
}

module dziurka2(){
translate([0,0,-3])cylinder(d=3.5,h=30);
}

module kolek(){
difference(){
cylinder(d=10,h=15);
translate([0,0,1])cylinder(d=2.5,h=15);
}
}

module kolek2(){
difference(){
cylinder(d=10,h=15);
translate([0,0,1])cylinder(d=1.5,h=15);
}
}
module rozdzielnica_zasilania(){
//difference(){
//cube([85,92,3]);
translate([5,4.44,-1])cylinder(d=4, h=10);
translate([8,36.44,-1])cylinder(d=4, h=10);
translate([8,64.44,-1])cylinder(d=4, h=10);
translate([7,83.44,-1])cylinder(d=4, h=10);
translate([80,4.44,-1])cylinder(d=4, h=10);
translate([77,21.44,-1])cylinder(d=4, h=10);
translate([77,49.44,-1])cylinder(d=4, h=10);
translate([79,82.44,-1])cylinder(d=4, h=10);

translate([5.5,11.75,-1])cube([7.25,7.25,10]);
translate([5.5,25.75,-1])cube([7.25,7.25,10]);
translate([5.5,39.75,-1])cube([7.25,7.25,10]);
translate([5.5,53.75,-1])cube([7.25,7.25,10]);
translate([5.5,67.75,-1])cube([7.25,7.25,10]);

translate([72.5,11,-1])cube([7.25,7.25,10]);
translate([72.5,25,-1])cube([7.25,7.25,10]);
translate([72.5,39,-1])cube([7.25,7.25,10]);
translate([72.5,53,-1])cube([7.25,7.25,10]);
translate([72.5,67,-1])cube([7.25,7.25,10]);

translate([12.5,5,-1])cube([60.5,81,10]);
//}
}

module baterie(){
cube([100,100,3]);
translate([2.78,3.25,2])kolek();
translate([47.78,5.25,2])kolek();
translate([85.78,39.25,2])kolek();
translate([47.78,91.75,2])kolek();

}



module rozdzielnica_sygnalu(){

cube([72,50,3]);
translate([30,3,2])kolek();
translate([26,37,2])kolek();
translate([52,35,2])kolek();

}

module silniki(){
cube([57,91,3]);
translate([6,6.17,2])kolek();
translate([39,5.17,2])kolek();
translate([18,87.17,2])kolek();
translate([53,87.17,2])kolek();

}

module zasilanie_5V(){
cube([59,51,3]);
translate([29,3.8,2])kolek();
translate([6,43.8,2])kolek();
translate([55,46.8,2])kolek();
}

module rpi(){
cube([95,61,3]);
translate([5,5,0]){
translate([3.5,3.5,2])kolek();
translate([3.5,52.5,2])kolek();
translate([61.5,3.5,2])kolek();
translate([61.5,52.5,2])kolek();
}
}

module router(){
cube([64,59,3]);
translate([-10,11,0])cube([30,10,3]);
translate([3.85+1.25,3.75+1.25,2])kolek2();
translate([2.85+1.25,52.5+1.25,2])kolek2();
translate([48.1+1.25,44.4+1.25,2])kolek2();
translate([-70,11,0])cube([60,10,17.5]);
}





difference(){
union(){
cube([205,165,30]);
difference(){
cube([205,165,26+5]);

}


translate([0-2,0-2,0])uchwyt1();
translate([205/2,0-2,0])uchwyt1();
translate([205+2,0-2,0])uchwyt1();

translate([0-2,165+2,0])uchwyt1();
translate([205/2,165+2,0])uchwyt1();
translate([205+2,165+2,0])uchwyt1();

translate([0-2,165/2,0])uchwyt1();
translate([205+2,165/2,0])uchwyt1();

translate([205+2,165-31,0])uchwyt1();
translate([205+2,165-59,0])uchwyt1();

translate([205-26,165+2,0])uchwyt1();
translate([205-49,165+2,0])uchwyt1();
translate([205-72,165+2,0])uchwyt1();
}
translate([1.4,1.4,26+4])cube([205-2.8,165-2.8,20]);
translate([3,3,3])cube([199,159,300]);
translate([200,165-20,30-5.5])cube([10,5.5,15.5]);
translate([200,165-48,30-5.5])cube([10,5.5,15.5]);
translate([200,165-76,30-5.5])cube([10,5.5,15.5]);

translate([0-2,0-2,0])dziurka1();
translate([205/2,0-2,0])dziurka1();
translate([205+2,0-2,0])dziurka1();

translate([0-2,165+2,0])dziurka1();
translate([205/2,165+2,0])dziurka1();
translate([205+2,165+2,0])dziurka1();

translate([0-2,165/2,0])dziurka1();
translate([205+2,165/2,0])dziurka1();

translate([205+2,165-31,0])dziurka1();
translate([205+2,165-59,0])dziurka1();

translate([205-26,165+2,0])dziurka1();
translate([205-49,165+2,0])dziurka1();
translate([205-72,165+2,0])dziurka1();



}
translate([5,160,0])rotate([0,0,-90])rpi();
translate([135,5,0])router();
translate([70,105,0])rozdzielnica_sygnalu();
translate([145,68,0])silniki();



translate([0,0,200]){
difference(){
union(){
cube([205,165,30]);
translate([0,0,-1])difference(){
translate([1.6,1.6,0])cube([199+3.2,159+3.2,5]);
translate([3,3,-1])cube([199,159,10]);
}

translate([0-2,0-2,0])uchwyt2();
translate([205/2,0-2,0])uchwyt2();
translate([205+2,0-2,0])uchwyt2();

translate([0-2,165+2,0])uchwyt2();
translate([205/2,165+2,0])uchwyt2();
translate([205+2,165+2,0])uchwyt2();

translate([0-2,165/2,0])uchwyt2();
translate([205+2,165/2,0])uchwyt2();

translate([205+2,165-31,0])uchwyt2();
translate([205+2,165-59,0])uchwyt2();

translate([205-26,165+2,0])uchwyt2();
translate([205-49,165+2,0])uchwyt2();
translate([205-72,165+2,0])uchwyt2();


}
translate([205-19,155,-11])cube([9,20,15]);
translate([205-42,155,-11])cube([9,20,15]);
translate([205-65,155,-11])cube([9,20,15]);
translate([205-88,155,-11])cube([9,20,15]);
translate([3,3,-3])cube([199,159,30]);
translate([5,68,7+17])rotate([0,0,0])rozdzielnica_zasilania();
translate([195-10,10+10,5+20])cylinder(d=19.5,h=10);

translate([0-2,0-2,0])dziurka2();
translate([205/2,0-2,0])dziurka2();
translate([205+2,0-2,0])dziurka2();

translate([0-2,165+2,0])dziurka2();
translate([205/2,165+2,0])dziurka2();
translate([205+2,165+2,0])dziurka2();

translate([0-2,165/2,0])dziurka2();
translate([205+2,165/2,0])dziurka2();

translate([205+2,165-31,0])dziurka2();
translate([205+2,165-59,0])dziurka2();

translate([205-26,165+2,0])dziurka2();
translate([205-49,165+2,0])dziurka2();
translate([205-72,165+2,0])dziurka2();


}

translate([199,45,30])rotate([0,0,0])rotate([0,180,0])baterie();

translate([5,5,30])rotate([0,180,270])zasilanie_5V();
}

