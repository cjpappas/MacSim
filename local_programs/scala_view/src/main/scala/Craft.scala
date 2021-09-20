class Craft (val topLeftLat: Double, topLeftLong: Double, scale: Double) {
  case class Loc(lat: Double, long: Double)
  var locs : Seq[Loc] = Seq()
  var alts: Seq[Double] = Seq()

  def logData(lat: Double, long: Double, alt: Double): Unit = {
    locs = Loc(lat, long) +: locs
    alts = alt +: alts
  }

  def render(): String = {
    val x = (locs.lift(0).map(x => x.long).getOrElse(topLeftLong) - topLeftLong)*scale
    val y = (locs.lift(0).map(y => y.lat).getOrElse(topLeftLong)  - topLeftLat )*scale
    val pixels: Seq[Seq[Char]] = Seq.fill(20){Seq.fill(20){' '}}
    pixels.map{row => row.foldRight("\n"){(c,rest) => c +: rest}}.foldRight(""){_++_}

  }
}