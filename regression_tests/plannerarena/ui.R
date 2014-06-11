library(shiny)

shinyUI(fluidPage(
    tags$head(
        tags$link(rel="stylesheet", type="text/css", href="ompl.css")),
    titlePanel("OMPL Planner Arena"),
  sidebarLayout(
    sidebarPanel(
        selectInput("plotType", label = h4("Plot type"),
            choices = list(
                "Regression plot" = 1,
                "Overall performance" = 2,
                "Progress" = 3
            )
        ),
        selectInput("problem", label = h4("Motion planning problem"),
            choices = list(
                "Circles",
                "Cubicles",
                "Twistycool"
            )
        ),
        conditionalPanel(
            condition = "input.plotType == 1 || input.plotType == 2",
            selectInput("attr", label = h4("Benchmark attribute"),
                choices = list(
                    "time",
                    "solved",
                    "approximate solution",
                    "solution difference",
                    "graph motions",
                    "graph states",
                    "solution length",
                    "simplification time",
                    "simplified solution length"
                )
            )
        ),
        conditionalPanel(
            condition = "input.plotType == 3",
            selectInput("progress", label = h4("Progress attribute"),
                choices = list(
                    "best cost",
                    "iterations",
                    "collision checks"
                )
            )
        ),
        conditionalPanel(
            condition = "input.plotType == 2 || input.plotType == 3",
            selectInput("version", label = h4("OMPL version"),
                choices = list(
                    "0.9.4",
                    "0.9.5",
                    "0.10.0",
                    "0.10.1",
                    "0.10.2",
                    "0.11.0",
                    "0.11.1",
                    "0.12.2",
                    "0.13.0",
                    "0.14.0",
                    "0.14.1",
                    "0.14.2"
                ),
                selected = "0.14.2"
            )
        ),
        checkboxGroupInput("planners", label = h4("Selected planners"),
            choices = list(
                "BKPIECE" = "BKPIECE1",
                "EST",
                "KPIECE" = "KPIECE1",
                "LazyPRM",
                "LBKPIECE" = "LBKPIECE1",
                "LBT-RRT" = "LBTRRT",
                "PDST",
                "PRM",
                "PRM*" = "PRMstar",
                "RRT",
                "RRT*" = "RRTstar",
                "RRTConnect",
                "SBL",
                "SPARS",
                "SPARS2" = "SPARStwo",
                "STRIDE",
                "T-RRT" = "TRRT"
            ),
            selected = list("KPIECE1","EST","RRT","PRM")
        )
    ),
    mainPanel(textOutput("debug"),plotOutput("plot"))
  )
))

