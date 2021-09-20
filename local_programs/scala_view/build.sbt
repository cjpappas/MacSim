ThisBuild / scalaVersion := "2.12.14"

lazy val builder = (project in file("."))
  .settings(
    name := "eps",
    libraryDependencies += "com.typesafe.akka" %% "akka-actor" % "2.6.15",
    libraryDependencies += "com.typesafe.akka" %% "akka-stream" % "2.6.15",
    libraryDependencies += "com.typesafe.akka" %% "akka-http" % "10.2.6",
    libraryDependencies += "org.json4s" %% "json4s-jackson" % "4.0.3",
    libraryDependencies += "net.team2xh" %% "scurses" % "1.0.1",
  )
