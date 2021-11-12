class Craft (val lat: Double, val long: Double, val alt: Double) {
  case class Loc(lat: Double, long: Double)
  var locs : Seq[Loc] = Seq()
  var alts: Seq[Double] = Seq()

  def logData(lat: Double, long: Double, alt: Double): Unit = {
    locs = Loc(lat, long) +: locs
    alts = alt +: alts
  }

}