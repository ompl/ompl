library(shiny)
library(ggplot2)
library(RSQLite)

defaultDatabase <- "www/benchmark.db"

sqlPlannerSelect <- function(name) sprintf('plannerConfigs.name = "geometric_%s"', name)
sqlVersionSelect <- function(version) sprintf('experiments.version = "%s"', version)

# limit file uploads to 10MB, suppress warnings
options(shiny.maxRequestSize = 10*1024^2, warn = -1)

shinyServer(function(input, output, session) {
    con <- reactive({
        if (is.null(input$database))
            database <- defaultDatabase
        else
            database <- input$database$datapath
        dbConnect(dbDriver("SQLite"), database)
    })
    plannerConfigs <- reactive({
        dbGetQuery(con(),
            "SELECT REPLACE(name, 'geometric_', '') as name, settings FROM plannerConfigs")
    })

    output$problemSelect <- renderUI({
        problems <- dbGetQuery(con(), "SELECT DISTINCT name FROM experiments")
        selectInput("problem",
            label = h4("Motion planning problem"),
            choices = paste(problems$name)
        )
    })
    output$versionSelect <- renderUI({
        versions <- dbGetQuery(con(), "SELECT version FROM experiments")
        if (input$plotType == 1)
            checkboxGroupInput("versions", label = h4("Selected versions"),
                choices = paste(versions$version),
                selected = paste(versions$version))
        else
            selectInput("version", label = h4("OMPL version"),
                choices = paste(versions$version),
                # select most recent version by default
                selected = paste(tail(versions$version, n=1))
            )
    })

    output$plannerSelect <- renderUI({
        p <- plannerConfigs()
        planners <- unique(unlist(p$name))
        # select first 4 planners (or all if there are less than 4)
        if (length(planners) < 4)
            selection <- planners
        else
            selection <- planners[1:4]
        checkboxGroupInput("planners", label = h4("Selected planners"),
            choices = planners, selected = selection
        )
    })


    runAttrs <- reactive({
        dbGetQuery(con(), "PRAGMA table_info(runs)")
    })

    output$attrSelect <- renderUI({
        if (input$plotType == 1 | input$plotType == 2)
        {
            attrs <- runAttrs()
            # strip off first 3 names, which correspond to internal id's
            attrNames <- gsub("_", " ", attrs$name[4:length(attrs$name)])
            if ('time' %in% attrNames)
                selection <- 'time'
            else
                selection <- NULL
            selectInput("attr", label = h4("Benchmark attribute"),
                choices = attrNames, selected = selection
            )
        }
    })

    output$progressSelect <- renderUI({
        if (input$plotType == 3)
        {
            progressAttrs <- dbGetQuery(con(), "PRAGMA table_info(progress)")
            # strip off first 2 names, which correspond to an internal id and time
            progressAttrNames <- gsub("_", " ", progressAttrs$name[3:length(progressAttrs$name)])
            if (is.na(progressAttrNames[1]))
                selectInput("progress", label = h4("Progress attribute"),
                    choices = c("Not available" = 0)
                )
            else
                selectInput("progress", label = h4("Progress attribute"),
                    choices = progressAttrNames
                )
        }
    })

    benchmarkInfo <- reactive({
        if (is.null(input$problem))
            return(NULL)
        if (is.null(input$version))
        {
            if (is.null(input$versions))
                return(NULL)
            else
                version <- tail(input$versions, n=1)
        }
        else
            version <- input$version

        query <- sprintf("SELECT * FROM experiments WHERE name=\"%s\" AND version=\"%s\"",
            input$problem, version)
        dbGetQuery(con(), query)
    })
    output$benchmarkInfo <- renderTable({ t(benchmarkInfo()) })

    output$plannerConfigs <- renderTable({ plannerConfigs() })

    plot <- reactive({
        if (is.null(input$problem))
            return(NULL)

        # regression plot
        if (input$plotType == 1) {
            if (is.null(input$versions))
                return(NULL)
            attr <- gsub(" ", "_", input$attr)
            query <- sprintf("SELECT plannerConfigs.name, runs.%s, experiments.version FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND (%s) AND (%s);",
                attr,
                input$problem,
                paste(sapply(input$planners, sqlPlannerSelect), collapse=" OR "),
                paste(sapply(input$versions, sqlVersionSelect), collapse=" OR "))
            data <- dbGetQuery(con(), query)
            # order by order listed in data frame (i.e., "0.9.*" before "0.10.*")
            data$version <- factor(data$version, unique(data$version))
            data$name <- sub("geometric_", "", data$name)
            p <- ggplot(data, aes_string(x = "version", y = attr, fill = "name", group = "name")) +
                # labels
                xlab('version') +
                ylab(input$attr) +
                theme(legend.title=element_blank()) +
                # plot mean and error bars
                stat_summary(fun.data = "mean_cl_boot", geom="bar", position = position_dodge()) +
                stat_summary(fun.data = "mean_cl_boot", geom="errorbar", position = position_dodge())
        }
        # plot of overall performance
        if (input$plotType == 2) {
            if (is.null(input$version))
                return(NULL)
            attr <- gsub(" ", "_", input$attr)
            query <- sprintf("SELECT plannerConfigs.name, runs.%s, experiments.version FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND (%s);",
                attr,
                input$problem,
                input$version,
                paste(sapply(input$planners, sqlPlannerSelect), collapse=" OR "))
            data <- dbGetQuery(con(), query)
            data$name <- sub("geometric_", "", data$name)
            attribs <- runAttrs()
            if (attribs$type[match(attr, attribs$name)] == "ENUM")
            {
                query <- sprintf("SELECT * FROM enums WHERE name=\"%s\";", attr)
                enum <- dbGetQuery(con(), query)
                val <- enum$value
                names(val) <- enum$description
                attrAsFactor <- factor(data[,match(attr, colnames(data))],
                    levels=enum$value, labels=enum$description)
                p <- qplot(name, data=data, geom="histogram", fill=attrAsFactor) +
                    # labels
                    xlab('planner') +
                    theme(legend.title=element_blank())
            }
            else
            {
                p <- ggplot(data, aes_string(x = "name", y = attr, group = "name")) +
                    # labels
                    xlab('planner') +
                    ylab(input$attr) +
                    theme(legend.position="none") +
                    # box plots for boolean, integer, and real-valued attributes
                    geom_boxplot(color = I("#3073ba"), fill = I("#99c9eb"))
            }
        }
        # progress plot
        if (input$plotType == 3) {
            if (is.null(input$version) | is.null(input$problem) | is.null(input$progress))
                return(NULL)
            attr <- gsub(" ", "_", input$progress)
            query <- sprintf("SELECT plannerConfigs.name, progress.time, progress.%s FROM progress INNER JOIN runs ON progress.runid = runs.id INNER JOIN plannerConfigs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND (%s);",
                attr,
                input$problem,
                input$version,
                paste(sapply(input$planners, sqlPlannerSelect), collapse=" OR "))
            data <- dbGetQuery(con(), query)
            data$name <- sub("geometric_", "", data$name)
            p <- ggplot(data, aes_string(x = "time", y = attr, group = "name", color = "name", fill = "name")) +
                # labels
                xlab('time (s)') +
                ylab(input$progress) +
                # semi-transparent points and a smooth interpolating curve
                geom_point(alpha=I(1/2)) + geom_smooth()
        }
        p
    })
    output$plot <- renderPlot({ print(plot()) })
    output$downloadPlot <- downloadHandler(filename = 'plot.pdf',
        content = function(file) {
            pdf(file=file, width=12, height=8)
            print(plot())
            dev.off()
        }
    )
})
