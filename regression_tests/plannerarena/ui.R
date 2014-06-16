library(shiny)

con <- dbConnect(dbDriver("SQLite"), "www/benchmark.db")
problems <- dbGetQuery(con, "SELECT DISTINCT name FROM experiments")
versions <- dbGetQuery(con, "SELECT version FROM experiments")
planners <- dbGetQuery(con, "SELECT DISTINCT REPLACE(name, 'geometric_', '') AS name FROM plannerConfigs")

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
            choices = paste(problems$name)
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
                choices = paste(versions$version),
                # select most recent version by default
                selected = paste(tail(versions$version, n=1))
            )
        ),
        checkboxGroupInput("planners", label = h4("Selected planners"),
            choices = paste(planners$name),
            selected = list("KPIECE1","EST","RRT","PRM")
        )
    ),
    mainPanel(
        tabsetPanel(
            tabPanel("Plot", span(downloadLink('downloadPlot', 'Download as PDF'), class="btn"), plotOutput("plot")),
            tabPanel("Benchmark Info", tableOutput("benchmarkInfo")),
            tabPanel("Planner Configurations", tableOutput("plannerConfigs"))
        )
    )
  )
))

