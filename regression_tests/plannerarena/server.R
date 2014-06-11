library(shiny)
library(ggplot2)
library(RSQLite)

con <- dbConnect(dbDriver("SQLite"), "www/benchmark.db")
sqlPlannerSelectAnyVersion <- function(name) sprintf('plannerConfigs.name LIKE "%%%%_%s-%%%%"', name)

shinyServer(function(input, output) {
    output$debug <- renderText({
    })

    output$plot <- renderPlot({
        if (input$plotType == 1) {
            attr <- gsub(" ", "_", input$attr)
            query <- sprintf("SELECT plannerConfigs.name, runs.%s FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid WHERE %s;",
                attr,
                paste(sapply(input$planners, sqlPlannerSelectAnyVersion), collapse=" OR "))
            data <- dbGetQuery(con, query)
            data$version <- sub("[a-z]+_[a-zA-Z]+-", "", data$name)
            # order by order listed in data frame (i.e., "0.9.*" before "0.10.*")
            data$version <- factor(data$version, unique(data$version))
            data$realname <- sub("-[0-9]+.[0-9]+.[0-9]+", "", sub("[a-z]+_", "", data$name))
            p <- ggplot(data, aes_string(x = "version", y = attr, color = "realname", fill = "realname", group = "name"))
            # labels
            p <- p + xlab('version') + ylab(input$attr) + theme(legend.title=element_blank())
            # plot mean and error bars
            p <- p + stat_summary(fun.data = "mean_cl_boot", geom="bar", position = position_dodge())
            p <- p + stat_summary(fun.data = "mean_cl_boot", geom="errorbar", position = position_dodge())
            print(p)
        }
        if (input$plotType == 2) {
            attr <- gsub(" ", "_", input$attr)
            sqlPlannerSelectSpecificVersion <- function(name) sprintf('plannerConfigs.name LIKE "%%%%_%s-%s"', name, input$version)
            query <- sprintf("SELECT plannerConfigs.name, runs.%s FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid WHERE plannerConfigs.name LIKE \"%%_%%-%s\";",
                attr,
                input$version,
                paste(sapply(input$planners, sqlPlannerSelectSpecificVersion), collapse=" OR "))
            data <- dbGetQuery(con, query)
            data$realname <- sub("-[0-9]+.[0-9]+.[0-9]+", "", sub("[a-z]+_", "", data$name))
            p <- ggplot(data, aes_string(x = "realname", y = attr, color = "realname", fill = "realname", group = "name"))
            # labels
            p <- p + xlab('planner') + ylab(input$attr) + theme(legend.title=element_blank())
            # plot mean and error bars
            p <- p + stat_summary(fun.data = "mean_cl_boot", geom="bar", position = position_dodge())
            p <- p + stat_summary(fun.data = "mean_cl_boot", geom="errorbar", position = position_dodge())
            print(p)
        }
        if (input$plotType == 3) {
                # TODO
        }
    })
})
