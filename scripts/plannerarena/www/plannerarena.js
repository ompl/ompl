shinyjs.refresh = function() {
    var div = document.createElement("div");
    div.innerHTML = "Benchmarking results are not available yet. This page will refresh automatically.";
    div.setAttribute("class", "shiny-output-error");
    document.getElementsByClassName("tab-content")[0].appendChild(div);

    window.setInterval(function(){
        location.reload();
    }, 5000);
}
