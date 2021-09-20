import org.json4s._
import org.json4s.jackson.JsonMethods._
import org.json4s.JsonDSL._

import net.team2xh.scurses.Scurses

object App {
  implicit val formats = DefaultFormats

  val craft: Craft = new Craft(-33.7225534, 150, 10)

  val processMessage: Function[String, Unit] = {(s: String) => 
    val data = parse(s)
    craft.logData((data \\ "latitude").extract[Double],
                  (data \\ "longitude").extract[Double],
                  (data \\ "altitude").extract[Double]
    )
    print("\u001b[2J")
    println(f"(${craft.locs(0).lat}%2.2f,${craft.locs(0).long}%2.2f)@${craft.alts(0)}%2.2f")
  }

  def main(args: Array[String]): Unit = {
    rosbridge.subscribe("/wamv/sensors/gps/gps/fix", processMessage)
  }
}