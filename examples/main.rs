use flowfield_nd::FlowField;

fn main() {
    let ff = FlowField::new(2, 10);
    for x in ff.enumerate() {
        dbg!(x);
    }
    println!("Done");

    for x in flowfield_nd::neighborhood(3) {
        println!("{:?}", x);
    }
}
