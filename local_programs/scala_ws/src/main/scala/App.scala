import org.json4s._
import org.json4s.jackson.JsonMethods._
import org.json4s.JsonDSL._

import net.team2xh.scurses.Scurses

object App {
  implicit val formats = DefaultFormats

  var craft: Option[Craft] = None

  val processMessage: Function[String, Unit] = {(s: String) => 
    val data = parse(s)
    craft match  {
      case None => craft = Some(new Craft((data \\ "latitude").extract[Double],
                                          (data \\ "longitude").extract[Double],
                                          (data \\ "altitude").extract[Double]
                                         ))
      case _ => {}
    }
    craft.get.logData((data \\ "latitude").extract[Double],
                      (data \\ "longitude").extract[Double],
                      (data \\ "altitude").extract[Double]
    )
    print("\u001b[2J")
    println(f"(${craft.get.locs(0).lat}%2.2f,${craft.get.locs(0).long}%2.2f)@${craft.get.alts(0)}%2.2f")
    println(f"x-drift: ${(craft.get.locs(0).lat - craft.get.lat) * 11139.0}%2.8f")
    println(f"y-drift: ${(craft.get.locs(0).long - craft.get.long) * 11139.0}%2.8f")
    println(f"z-drift: ${(craft.get.alts(0) - craft.get.alt)}%2.8f")
  }

  def main(args: Array[String]): Unit = {
    rosbridge.subscribe("/wamv/sensors/gps/gps/fix", processMessage)
  }
}