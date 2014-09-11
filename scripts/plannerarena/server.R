library(shiny)
library(ggplot2)
library(RSQLite)

defaultDatabase <- "www/benchmark.db"

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

versionSelectWidget <- function(con, name, checkbox) {
    versions <- dbGetQuery(con, "SELECT DISTINCT version FROM experiments")
    versions <- versions$version
    if (checkbox)
        widget <- checkboxGroupInput(name, label = h4("Selected versions"),
            choices = versions,
            selected = versions)
    else
        widget <- selectInput(name, label = h4("OMPL version"),
            choices = versions,
            # select most recent version by default
            selected = tail(versions, n=1))
    conditionalDisable(widget, length(versions) < 2)
}

plannerNameMapping <- function(fullname) {
    sub("control_", " ", sub("geometric_", "", fullname))
}
plannerSelectWidget <- function(con, name, problem, version) {
    planners <- dbGetQuery(con, "SELECT DISTINCT name, settings FROM plannerConfigs")
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

# limit file uploads to 30MB, suppress warnings
options(shiny.maxRequestSize = 30*1024^2, warn = -1)

shinyServer(function(input, output, session) {
    con <- reactive({
        if (is.null(input$database))
            database <- defaultDatabase
        else
            database <- input$database$datapath
        dbConnect(dbDriver("SQLite"), database)
    })

    output$perfProblemSelect <- renderUI({ problemSelectWidget(con(), "perfProblem") })
    output$progProblemSelect <- renderUI({ problemSelectWidget(con(), "progProblem") })
    output$regrProblemSelect <- renderUI({ problemSelectWidget(con(), "regrProblem") })

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

    output$perfAttrSelect <- renderUI({ perfAttrSelectWidget(con(), "perfAttr") })
    output$regrAttrSelect <- renderUI({ perfAttrSelectWidget(con(), "regrAttr") })

    progAttrs <- reactive({
        progressAttrs <- dbGetQuery(con(), "PRAGMA table_info(progress)")
        # strip off first 2 names, which correspond to an internal id and time
        gsub("_", " ", progressAttrs$name[3:length(progressAttrs$name)])
    })
    output$progAttrSelect <- renderUI({
        attrs <- progAttrs()
        list(
            conditionalDisable(selectInput("progress", label = h4("Progress attribute"),
                choices = attrs
            ), length(attrs) < 2),
            checkboxInput("progressShowMeasurements", label = h6("Show individual measurements")),
            sliderInput("progressOpacity", label = h6("Measurement opacity"), 0, 100, 50)
        )
    })

    benchmarkInfo <- reactive({
        if (!is.null(input$perfVersion))
        {
            query <- sprintf("SELECT * FROM experiments WHERE name=\"%s\" AND version=\"%s\"",
                input$perfProblem, input$perfVersion)
            dbGetQuery(con(), query)
        }
    })
    output$benchmarkInfo <- renderTable({ t(benchmarkInfo()) })
    output$plannerConfigs <- renderTable({
        query <- sprintf("SELECT DISTINCT plannerConfigs.name, plannerConfigs.settings FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid AND plannerConfigs.id = runs.plannerid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\";",
            input$perfProblem, input$perfVersion)
        dbGetQuery(con(), query)
    })

    # plot of overall performance
    perfPlot <- reactive({
        attr <- gsub(" ", "_", input$perfAttr)
        query <- sprintf("SELECT plannerConfigs.name AS name, runs.%s, experiments.version FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid AND plannerConfigs.id = runs.plannerid WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND (%s);",
            attr,
            input$perfProblem,
            input$perfVersion,
            paste(sapply(input$perfPlanners, sqlPlannerSelect), collapse=" OR "))
        data <- dbGetQuery(con(), query)
        data$name <- factor(data$name, unique(data$name), labels = sapply(unique(data$name), plannerNameMapping))
        attribs <- perfAttrs(con())
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
                scale_x_discrete('planner', labels = names(input$perfProblem))
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
    output$perfDownloadPlot <- downloadHandler(filename = 'plot.pdf',
        content = function(file) {
            pdf(file=file, width=12, height=8)
            print(perfPlot())
            dev.off()
        }
    )

    # progress plot
    progPlot <- reactive({
        attr <- gsub(" ", "_", input$progress)
        query <- sprintf("SELECT plannerConfigs.name AS name, progress.time, progress.%s FROM plannerConfigs INNER JOIN runs ON plannerConfigs.id = runs.plannerid INNER JOIN experiments ON experiments.id = runs.experimentid AND plannerConfigs.id = runs.plannerid INNER JOIN progress ON progress.runid = runs.id WHERE experiments.name=\"%s\" AND experiments.version=\"%s\" AND progress.%s IS NOT NULL AND (%s);",
            attr,
            input$progProblem,
            input$progVersion,
            attr,
            paste(sapply(input$progPlanners, sqlPlannerSelect), collapse=" OR "))
        data <- dbGetQuery(con(), query)
        data$name <- factor(data$name, unique(data$name), labels = sapply(unique(data$name), plannerNameMapping))
        p <- ggplot(data, aes_string(x = "time", y = attr, group = "name", color = "name", fill = "name")) +
            # labels
            xlab('time (s)') +
            ylab(input$progress) +
            # smooth interpolating curve
            geom_smooth(method = "gam")
        # optionally, add individual measurements as semi-transparent points
        if (input$progressShowMeasurements)
            p <- p + geom_point(alpha=I(input$progressOpacity / 100))
        p
    })
    output$progPlot <- renderPlot({
        validate(
            need(input$progVersion, 'Select a version'),
            need(input$progProblem, 'Select a problem'),
            need(input$progress, 'Select a benchmark attribute'),
            need(input$progPlanners, 'Select some planners')
        )
        print(progPlot())
    })
    output$progDownloadPlot <- downloadHandler(filename = 'plot.pdf',
        content = function(file) {
            pdf(file=file, width=12, height=8)
            print(progPlot())
            dev.off()
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
            xlab('version') +
            ylab(input$regrAttr) +
            theme(legend.title = element_blank()) +
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
    output$regrDownloadPlot <- downloadHandler(filename = 'plot.pdf',
        content = function(file) {
            pdf(file=file, width=12, height=8)
            print(regrPlot())
            dev.off()
        }
    )

    output$progressPage <- renderUI({
        validate(
            need(!is.na(progAttrs()[1]), "There is no progress data in this database")
        )
        sidebarLayout(
            sidebarPanel(
                uiOutput("progProblemSelect"),
                uiOutput("progAttrSelect"),
                uiOutput("progVersionSelect"),
                uiOutput("progPlannerSelect")
            ),
            mainPanel(
                span(downloadLink('progDownloadPlot', 'Download plot as PDF'), class="btn"),
                plotOutput("progPlot")
            )
        )
    })

    output$regressionPage <- renderUI({
        validate(
            need(numVersions(con())>1, "Only one version of OMPL was used for the benchmarks")
        )
        sidebarLayout(
            sidebarPanel(
                uiOutput("regrProblemSelect"),
                uiOutput("regrAttrSelect"),
                uiOutput("regrVersionSelect"),
                uiOutput("regrPlannerSelect")
            ),
            mainPanel(
                span(downloadLink('regrDownloadPlot', 'Download plot as PDF'), class="btn"),
                plotOutput("regrPlot")
            )
        )
    })
})
