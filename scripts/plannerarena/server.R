library(shiny)
library(ggplot2)
library(RSQLite)
library(reshape2)

defaultDatabase <- "www/benchmark.db"

noDatabaseText <- "No database loaded yet. Upload one by clicking on “Change database”."

disable <- function(x) {
  if (inherits(x, 'shiny.tag')) {
    if (x$name %in% c('input', 'select', 'label'))
      x$attribs$disabled <- 'disabled'
    x$children <- disable(x$children)
  }
  else if (is.list(x) && length(x) > 0) {
    for (i in 1:length(x))
      x[[i]] <- disable(x[[i]])
  }
  x
}
conditionalDisable <- function(widget, condition) {
    if (condition)
        disable(widget)
    else
        widget
}

sqlPlannerSelect <- function(name) sprintf('plannerConfigs.name = "%s"', name)
sqlVersionSelect <- function(version) sprintf('experiments.version = "%s"', version)
problemSelectWidget <- function(con, name) {
    problems <- dbGetQuery(con, "SELECT DISTINCT name FROM experiments")
    problems <- problems$name
    widget <- selectInput(name,
        label = h4("Motion planning problem"),
        choices = problems)
    conditionalDisable(widget, length(problems) < 2)
}

numVersions <- function(con) {
    dbGetQuery(con, "SELECT COUNT(DISTINCT version) FROM experiments")
}

stripOMPLPrefix <- function(str) {
    sub("OMPL ", "", str)
}
versionSelectWidget <- function(con, name, checkbox) {
    versions <- dbGetQuery(con, "SELECT DISTINCT version FROM experiments")
    versions <- versions$version
    if (checkbox)
    {
        # strip "OMPL " prefix, so we can fit more labels on the X-axis
        versions <- sapply(stripLibnamePrefix, versions)
        widget <- checkboxGroupInput(name, label = h4("Selected versions"),
            choices = versions,
            selected = versions)
    }
    else
        widget <- selectInput(name, label = h4("Version"),
            choices = versions,
            # select most recent version by default
            selected = tail(versions, n=1))
    conditionalDisable(widget, length(versions) < 2)
}

plannerNameMapping <- function(fullname) {
    sub("control_", " ", sub("geometric_", "", fullname))
}
plannerSelectWidget <- function(con, name, problem, version) {
    query <- sprintf("SELECT DISTINCT plannerConfigs.name AS name FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\";", problem, version)
    planners <- dbGetQuery(con, query)
    planners <- unique(unlist(planners$name))
    names(planners) <- sapply(planners, plannerNameMapping)
    # select first 4 planners (or all if there are less than 4)
    if (length(planners) < 4)
        selection <- planners
    else
        selection <- planners[1:4]
    conditionalDisable(checkboxGroupInput(name, label = h4("Selected planners"),
        choices = planners, selected = selection), length(planners) < 2)
}

perfAttrs <- function(con) {
    dbGetQuery(con, "PRAGMA table_info(runs)")
}
perfAttrSelectWidget <- function(con, name) {
    attrs <- perfAttrs(con)
    # strip off first 3 names, which correspond to internal id's
    attrNames <- gsub("_", " ", attrs$name[4:length(attrs$name)])
    if ('time' %in% attrNames)
        selection <- 'time'
    else
        selection <- NULL
    selectInput(name, label = h4("Benchmark attribute"),
        choices = attrNames, selected = selection)
}

hasProgressData <- function(con) {
    count <- dbGetQuery(con, "SELECT COUNT(*) FROM progress")
    count > 0
}

# limit file uploads to 30MB, suppress warnings
options(shiny.maxRequestSize = 30*1024^2, warn = -1)

shinyServer(function(input, output, session) {
    con <- reactive({
        if (is.null(input$database) || is.null(input$database$datapath))
            database <- defaultDatabase
        else
            database <- input$database$datapath
        #return(normalizePath(database))
        if (file.exists(database))
            dbConnect(dbDriver("SQLite"), database)
        else
            NULL
    })

    # Go straight to the database upload page if there is no default database
    observe({
        if (is.null(con()))
            updateTabsetPanel(session, "navbar", selected = "database")
    })


    output$perfProblemSelect <- renderUI({ problemSelectWidget(con(), "perfProblem") })
    output$progProblemSelect <- renderUI({ problemSelectWidget(con(), "progProblem") })
    output$regrProblemSelect <- renderUI({ problemSelectWidget(con(), "regrProblem") })

    output$perfAttrSelect <- renderUI({
        list(
            perfAttrSelectWidget(con(), "perfAttr"),
            checkboxInput('perfShowAdvOptions', 'Show advanced options', FALSE),
            conditionalPanel(condition = 'input.perfShowAdvOptions',
                div(class="well well-light",
                    checkboxInput("perfShowAsCDF", label = "Show as cumulative distribution function"),
                    checkboxInput("perfShowSimplified", label = "Include results after simplification")
                )
            )
        )
    })
    output$regrAttrSelect <- renderUI({ perfAttrSelectWidget(con(), "regrAttr") })
    output$progAttrSelect <- renderUI({
        progressAttrs <- dbGetQuery(con(), "PRAGMA table_info(progress)")
        # strip off first 2 names, which correspond to an internal id and time
        attrs <- gsub("_", " ", progressAttrs$name[3:length(progressAttrs$name)])
        list(
            conditionalDisable(selectInput("progress", label = h4("Progress attribute"),
                choices = attrs
            ), length(attrs) < 2),
            div(class="well well-light",
                checkboxInput("progressShowMeasurements", label = "Show individual measurements"),
                sliderInput("progressOpacity", label = "Measurement opacity", 0, 100, 50)
            )
        )
    })

    output$perfVersionSelect <- renderUI({ versionSelectWidget(con(), "perfVersion", FALSE) })
    output$progVersionSelect <- renderUI({ versionSelectWidget(con(), "progVersion", FALSE) })
    output$regrVersionSelect <- renderUI({ versionSelectWidget(con(), "regrVersions", TRUE) })

    output$perfPlannerSelect <- renderUI({
        validate(
            need(input$perfProblem, 'Select a problem'),
            need(input$perfVersion, 'Select a version')
        )
        plannerSelectWidget(con(), "perfPlanners", input$perfProblem, input$perfVersion)
    })
    output$progPlannerSelect <- renderUI({
        validate(
            need(input$progProblem, 'Select a problem'),
            need(input$progVersion, 'Select a version')
        )
        plannerSelectWidget(con(), "progPlanners", input$progProblem, input$progVersion)
    })
    output$regrPlannerSelect <- renderUI({
        validate(
            need(input$regrProblem, 'Select a problem'),
            need(input$regrVersions, 'Select a version')
        )
        plannerSelectWidget(con(), "regrPlanners", input$regrProblem, tail(input$regrVersions, n=1))
    })


    output$benchmarkInfo <- renderTable({
        validate(need(con(), noDatabaseText))
        validate(need(input$perfVersion, "Select a version on the “Overall performance” page"))
        query <- sprintf("SELECT * FROM experiments WHERE name=\"%s\" AND version=\"%s\"",
            input$perfProblem, input$perfVersion)
        data <- dbGetQuery(con(), query)
        t(data)
    }, include.colnames=FALSE)
    output$plannerConfigs <- renderTable({
        query <- sprintf("SELECT DISTINCT plannerConfigs.name, plannerConfigs.settings FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\";",
            input$perfProblem, input$perfVersion)
        dbGetQuery(con(), query)
    }, include.rownames=FALSE)

    # font selection
    fontSelection <- reactive({
        element_text(family = input$fontFamily, size = input$fontSize)
    })

    # plot of overall performance
    perfPlot <- reactive({
        attribs <- perfAttrs(con())
        attr <- gsub(" ", "_", input$perfAttr)
        simplifiedAttr <- paste("simplified", attr, sep="_")
        includeSimplifiedAttr <- input$perfShowSimplified && simplifiedAttr %in% attribs$name
        if (includeSimplifiedAttr)
            query <- sprintf("SELECT plannerConfigs.name AS planner, runs.%s, runs.%s FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND (%s);",
                attr,
                simplifiedAttr,
                input$perfProblem,
                input$perfVersion,
                paste(sapply(input$perfPlanners, sqlPlannerSelect), collapse=" OR "))
        else
            query <- sprintf("SELECT plannerConfigs.name AS planner, runs.%s FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND (%s);",
                attr,
                input$perfProblem,
                input$perfVersion,
                paste(sapply(input$perfPlanners, sqlPlannerSelect), collapse=" OR "))
        data <- dbGetQuery(con(), query)
        data$planner <- factor(data$planner, unique(data$planner), labels = sapply(unique(data$planner), plannerNameMapping))
        if (attribs$type[match(attr, attribs$name)] == "ENUM")
        {
            query <- sprintf("SELECT * FROM enums WHERE name=\"%s\";", attr)
            enum <- dbGetQuery(con(), query)
            val <- enum$value
            names(val) <- enum$description
            attrAsFactor <- factor(data[,match(attr, colnames(data))],
                levels=enum$value, labels=enum$description)
            p <- qplot(planner, data=data, geom="histogram", fill=attrAsFactor) +
                # labels
                theme(legend.title = element_blank(), text = fontSelection())
        }
        else
        {
            if (includeSimplifiedAttr)
            {
                data <- melt(data, id.vars='planner', measure.vars=c(attr, simplifiedAttr))
                if (input$perfShowAsCDF)
                    p <- ggplot(data, aes(x = value, color = planner,
                        group = interaction(planner, variable), linetype=variable)) +
                        # labels
                        xlab(input$perfAttr) +
                        ylab('cumulative probability') +
                        theme(text = fontSelection()) +
                        # empirical cumulative distribution function
                        stat_ecdf(size = 1) +
                        scale_linetype_discrete(name = "", labels = c("before simplification", "after simplification"))
                else
                    p <- ggplot(data, aes(x=planner, y=value, color=variable, fill=variable)) +
                        # labels
                        ylab(input$perfAttr) +
                        theme(legend.title = element_blank(), text = fontSelection()) +
                        geom_boxplot() +
                        scale_fill_manual(values = c("#99c9eb", "#ebc999"),
                            labels = c("before simplification", "after simplification")) +
                        scale_color_manual(values =c("#3073ba", "#ba7330"),
                            labels = c("before simplification", "after simplification"))
            }
            else
            {
                if (input$perfShowAsCDF)
                    p <- ggplot(data, aes_string(x = attr, group = "planner", color = "planner")) +
                        # labels
                        xlab(input$perfAttr) +
                        ylab('cumulative probability') +
                        theme(text = fontSelection()) +
                        # empirical cumulative distribution function
                        stat_ecdf(size = 1)
                else
                    p <- ggplot(data, aes_string(x = "planner", y = attr, group = "planner")) +
                        # labels
                        ylab(input$perfAttr) +
                        theme(legend.position = "none", text = fontSelection()) +
                        # box plots for boolean, integer, and real-valued attributes
                        geom_boxplot(color = I("#3073ba"), fill = I("#99c9eb"))
            }
        }
        p
    })
    output$perfPlot <- renderPlot({
        validate(
            need(input$perfVersion, 'Select a version'),
            need(input$perfProblem, 'Select a problem'),
            need(input$perfAttr, 'Select a benchmark attribute'),
            need(input$perfPlanners, 'Select some planners')
        )
        print(perfPlot())
    })
    output$perfDownloadPlot <- downloadHandler(filename = 'perfplot.pdf',
        content = function(file) {
            pdf(file=file, width=input$paperWidth, height=input$paperHeight)
            print(perfPlot())
            dev.off()
        }
    )
    output$perfDownloadRdata <- downloadHandler(filename = 'perfplot.RData',
        content = function(file) {
            perfplot <- perfPlot()
            save(perfplot, file = file)
        }
    )
    output$perfMissingDataTable <- renderTable({
        validate(
            need(input$perfVersion, 'Select a version'),
            need(input$perfProblem, 'Select a problem'),
            need(input$perfAttr, 'Select a benchmark attribute'),
            need(input$perfPlanners, 'Select some planners')
        )
        attr <- gsub(" ", "_", input$perfAttr)
        query <- sprintf("SELECT plannerConfigs.name AS planner, SUM(runs.%s IS NULL) AS missing, COUNT(*) AS total FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND (%s) GROUP BY plannerConfigs.name;",
            attr,
            input$perfProblem,
            input$perfVersion,
            paste(sapply(input$perfPlanners, sqlPlannerSelect), collapse=" OR "))
        data <- dbGetQuery(con(), query)
        data$planner <- factor(data$planner, unique(data$planner), labels = sapply(unique(data$planner), plannerNameMapping))
        data
    }, include.rownames=FALSE)

    # progress plot
    progPlotData <- reactive({
        validate(
            need(input$progVersion, 'Select a version'),
            need(input$progProblem, 'Select a problem'),
            need(input$progress, 'Select a benchmark attribute'),
            need(input$progPlanners, 'Select some planners')
        )
        attr <- gsub(" ", "_", input$progress)
        query <- sprintf("SELECT plannerConfigs.name AS planner, progress.time, progress.%s FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid INNER JOIN progress ON progress.runid = runs.id WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND progress.%s IS NOT NULL AND (%s);",
            attr,
            input$progProblem,
            input$progVersion,
            attr,
            paste(sapply(input$progPlanners, sqlPlannerSelect), collapse=" OR "))
        data <- dbGetQuery(con(), query)
        data$planner <- factor(data$planner, unique(data$planner), labels = sapply(unique(data$planner), plannerNameMapping))
        data
    })
    progPlot <- reactive({
        attr <- gsub(" ", "_", input$progress)
        data <- progPlotData()
        validate(need(nrow(data) > 0, 'No progress data available; select a different benchmark, progress attribute, or planners.'))
        p <- ggplot(data, aes_string(x = "time", y = attr, group = "planner", color = "planner", fill = "planner")) +
            # labels
            xlab('time (s)') +
            ylab(input$progress) +
            theme(text = fontSelection()) +
            # smooth interpolating curve
            geom_smooth(method = "gam") +
            coord_cartesian(xlim = c(0, trunc(max(data$time))))
        # optionally, add individual measurements as semi-transparent points
        if (input$progressShowMeasurements)
            p <- p + geom_point(alpha=I(input$progressOpacity / 100))
        p
    })
    output$progPlot <- renderPlot({ progPlot() })
    progNumMeasurementsPlot <- reactive({
        data <- progPlotData()
        if (nrow(data) > 0)
        {
            p <- ggplot(data, aes(x = time, group = planner, color = planner)) +
                # labels
                xlab('time (s)') +
                ylab(sprintf("# measurements for %s", input$progress)) +
                theme(text = fontSelection()) +
                geom_freqpoly(binwidth=1) +
                coord_cartesian(xlim = c(0, trunc(max(data$time))))
            p
        }
    })
    output$progNumMeasurementsPlot <- renderPlot({ progNumMeasurementsPlot() })
    output$progDownloadPlot <- downloadHandler(filename = 'progplot.pdf',
        content = function(file) {
            pdf(file=file, width=input$paperWidth, height=input$paperHeight)
            print(progPlot())
            print(progNumMeasurementsPlot())
            dev.off()
        }
    )
    output$progDownloadRdata <- downloadHandler(filename = 'progplot.RData',
        content = function(file) {
            progplot <- progPlot()
            prognummeasurementsplot <- progNumMeasurementsPlot()
            save(progplot, prognummeasurementsplot, file = file)
        }
    )

    # regression plot
    regrPlot <- reactive({
        attr <- gsub(" ", "_", input$regrAttr)
        query <- sprintf("SELECT plannerConfigs.name AS name, runs.%s, experiments.version FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid WHERE experiments.name=\"%s\" AND (%s) AND (%s);",
            attr,
            input$regrProblem,
            paste(sapply(input$regrPlanners, sqlPlannerSelect), collapse=" OR "),
            paste(sapply(input$regrVersions, sqlVersionSelect), collapse=" OR "))
        data <- dbGetQuery(con(), query)
        # order by order listed in data frame (i.e., "0.9.*" before "0.10.*")
        data$version <- factor(data$version, unique(data$version))
        data$name <- factor(data$name, unique(data$name), labels = sapply(unique(data$name), plannerNameMapping))
        ggplot(data, aes_string(x = "version", y = attr, fill = "name", group = "name")) +
            # labels
            ylab(input$regrAttr) +
            theme(legend.title = element_blank(), text = fontSelection()) +
            # plot mean and error bars
            stat_summary(fun.data = "mean_cl_boot", geom="bar", position = position_dodge()) +
            stat_summary(fun.data = "mean_cl_boot", geom="errorbar", position = position_dodge())
    })
    output$regrPlot <- renderPlot({
        validate(
            need(input$regrVersions, 'Select a version'),
            need(input$regrProblem, 'Select a problem'),
            need(input$regrAttr, 'Select a benchmark attribute'),
            need(input$regrPlanners, 'Select some planners')
        )
        print(regrPlot())
    })
    output$regrDownloadPlot <- downloadHandler(filename = 'regrplot.pdf',
        content = function(file) {
            pdf(file=file, width=input$paperWidth, height=input$paperHeight)
            print(regrPlot())
            dev.off()
        }
    )
    output$regrDownloadRdata <- downloadHandler(filename = 'regrplot.RData',
        content = function(file) {
            regrplot <- regrPlot()
            save(regrplot, file = file)
        }
    )

    output$performancePage <- renderUI({
        validate(need(con(), noDatabaseText))
        sidebarLayout(
            sidebarPanel(
                uiOutput("perfProblemSelect"),
                uiOutput("perfAttrSelect"),
                uiOutput("perfVersionSelect"),
                uiOutput("perfPlannerSelect")
            ),
            mainPanel(
                span(downloadLink('perfDownloadPlot', 'Download plot as PDF'), class="btn"),
                span(downloadLink('perfDownloadRdata', 'Download plot as RData'), class="btn"),
                plotOutput("perfPlot"),
                h4("Number of missing data points out of the total number of runs per planner"),
                tableOutput("perfMissingDataTable")
            )
        )
    })
    output$progressPage <- renderUI({
        validate(need(con(), noDatabaseText))
        validate(need(hasProgressData(con()), "There is no progress data in this database."))
        sidebarLayout(
            sidebarPanel(
                uiOutput("progProblemSelect"),
                uiOutput("progAttrSelect"),
                uiOutput("progVersionSelect"),
                uiOutput("progPlannerSelect")
            ),
            mainPanel(
                span(downloadLink('progDownloadPlot', 'Download plot as PDF'), class="btn"),
                span(downloadLink('progDownloadRdata', 'Download plot as RData'), class="btn"),
                plotOutput("progPlot"),
                plotOutput("progNumMeasurementsPlot")
            )
        )
    })

    output$regressionPage <- renderUI({
        validate(need(con(), noDatabaseText))
        validate(need(numVersions(con())>1, "Only one version of OMPL was used for the benchmarks."))
        sidebarLayout(
            sidebarPanel(
                uiOutput("regrProblemSelect"),
                uiOutput("regrAttrSelect"),
                uiOutput("regrVersionSelect"),
                uiOutput("regrPlannerSelect")
            ),
            mainPanel(
                span(downloadLink('regrDownloadPlot', 'Download plot as PDF'), class="btn"),
                span(downloadLink('regrDownloadRdata', 'Download plot as RData'), class="btn"),
                plotOutput("regrPlot")
            )
        )
    })

    output$dbinfoPage <- renderUI({
        validate(need(con(), noDatabaseText))
        tabsetPanel(
            tabPanel("Benchmark setup",  tableOutput("benchmarkInfo")),
            tabPanel("Planner Configurations", tableOutput("plannerConfigs"))
        )
    })
})
