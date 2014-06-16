library(shiny)
library(ggplot2)
library(RSQLite)

con <- dbConnect(dbDriver("SQLite"), "www/benchmark.db")
sqlPlannerSelect <- function(name) sprintf('plannerConfigs.name = "geometric_%s"', name)
plannerConfigs <- dbGetQuery(con,
    "SELECT REPLACE(name, 'geometric_', '') as name, settings FROM plannerConfigs")


shinyServer(function(input, output) {
    benchmarkInfo <- reactive({
        query <- sprintf("SELECT * FROM experiments WHERE name=\"%s\" AND version=\"%s\"",
            input$problem,
            input$version)
        res <- dbGetQuery(con, query)
    })
    output$benchmarkInfo <- renderTable({ t(benchmarkInfo()) })

    output$plannerConfigs <- renderTable({ plannerConfigs })

    output$plot <- renderPlot({
        # regression plot
        if (input$plotType == 1) {
            attr <- gsub(" ", "_", input$attr)
            query <- sprintf("SELECT plannerConfigs.name, runs.%s, experiments.version FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND (%s);",
                attr,
                input$problem,
                paste(sapply(input$planners, sqlPlannerSelect), collapse=" OR "))
            data <- dbGetQuery(con, query)
            # order by order listed in data frame (i.e., "0.9.*" before "0.10.*")
            data$version <- factor(data$version, unique(data$version))
            data$name <- sub("geometric_", "", data$name)
            p <- ggplot(data, aes_string(x = "version", y = attr, color = "name", fill = "name", group = "name")) +
                # labels
                xlab('version') +
                ylab(input$attr) +
                theme(legend.title=element_blank()) +
                # plot mean and error bars
                stat_summary(fun.data = "mean_cl_boot", geom="bar", position = position_dodge()) +
                stat_summary(fun.data = "mean_cl_boot", geom="errorbar", position = position_dodge())
            print(p)
        }
        # plot of overall performance
        if (input$plotType == 2) {
            attr <- gsub(" ", "_", input$attr)
            query <- sprintf("SELECT plannerConfigs.name, runs.%s, experiments.version FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND (%s);",
                attr,
                input$problem,
                input$version,
                paste(sapply(input$planners, sqlPlannerSelect), collapse=" OR "))
            data <- dbGetQuery(con, query)
            data$name <- sub("geometric_", "", data$name)
            p <- ggplot(data, aes_string(x = "name", y = attr, color = "name", fill = "name", group = "name")) +
                # labels
                xlab('planner') +
                ylab(input$attr) +
                theme(legend.title=element_blank()) +
                # plot mean and error bars
                stat_summary(fun.data = "mean_cl_boot", geom="bar", position = position_dodge()) +
                stat_summary(fun.data = "mean_cl_boot", geom="errorbar", position = position_dodge())
            print(p)
        }
        # progress plot
        if (input$plotType == 3) {
            # TODO
        }
    })
})
