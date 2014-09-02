library(shiny)

shinyUI(fluidPage(
    tags$head(tags$link(rel="stylesheet", type="text/css", href="ompl.css")),
    titlePanel("OMPL Planner Arena"),
    sidebarLayout(
        sidebarPanel(
            fileInput("database",
                label = h4("Benchmark database"),
                accept = c("application/x-sqlite3", ".db")
            ),
            selectInput("plotType", label = h4("Plot type"),
                choices = list(
                    "Regression plot" = 1,
                    "Overall performance" = 2,
                    "Progress" = 3
                ),
                selected = 2
            ),
            uiOutput("problemSelect"),
            uiOutput("attrSelect"),
            uiOutput("progressSelect"),
            uiOutput("versionSelect"),
            uiOutput("plannerSelect")
        ),
        mainPanel(
            tabsetPanel(
                tabPanel("Plot",
                    span(downloadLink('downloadPlot', 'Download as PDF'), class="btn"),
                    plotOutput("plot")),
                tabPanel("Benchmark Info", tableOutput("benchmarkInfo")),
                tabPanel("Planner Configurations", tableOutput("plannerConfigs"))
            )
        )
    )
))

