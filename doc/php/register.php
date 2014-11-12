<?php
$register_email ='mmoll@rice.edu';

session_start();
$errors = '';
$email = '';
$affiliation = '';
$research = '';
$education = '';
$industry = '';
$message = '';

if(isset($_POST['submit']))
{

    $email = $_POST['email'];
    $affiliation = $_POST['affiliation'];
    $research = $_POST['research'];
    $education = $_POST['education'];
    $industry = $_POST['industry'];
    $message = $_POST['message'];

    ///------------Do Validations-------------
    if(empty($affiliation))
    {
        $errors .= "\n \"School/employer\" is a required field. ";
    }
    if(IsInjected($email))
    {
        $errors .= "\n Bad email value!";
    }

    if(empty($errors))
    {
        //send the email
        $to = $register_email;
        $subject="=== OMPL registration ===";
        $from = $register_email;
        $ip = isset($_SERVER['REMOTE_ADDR']) ? $_SERVER['REMOTE_ADDR'] : '';

        $body = "A user submitted the contact form:\n".
            "Email: $email\n".
            "Affiliation: $affiliation\n".
            "Research: $research\nEducation: $education\nIndustry: $industry\n".
            "Message: $message\n".
            "IP: $ip\n";

        $headers = "From: $from \r\n";
        $headers .= "Reply-To: $visitor_email \r\n";

        mail($to, $subject, $body,$headers);

        header('Location: thank-you.html');
    }
}

// Function to validate against any email injection attempts
function IsInjected($str)
{
  $injections = array('(\n+)',
              '(\r+)',
              '(\t+)',
              '(%0A+)',
              '(%0D+)',
              '(%08+)',
              '(%09+)'
              );
  $inject = join('|', $injections);
  $inject = "/$inject/i";
  if(preg_match($inject,$str))
    {
    return true;
  }
  else
    {
    return false;
  }
}
?>
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Register</title>
  <link href="../css/bootstrap.min.css" rel="stylesheet">
  <link href="../css/ompl.css" rel="stylesheet">
<!-- define some style elements-->
<style>
.err { color: #f00; }
textarea { resize: none; }
</style>
</head>

<body style="padding:0px 20px 0px 20px;">
<?php
if(!empty($errors)){
echo "<p class='err'>".nl2br($errors)."</p>";
}
?>
<div class="container">
<div class="row">
<p>Registration for OMPL is completely voluntary. We will not spam you with emails. Work on OMPL is funded in part by the National Science Foundation (NSF). It is extremely helpful for us if we can report to the NSF where and how our work is being used. Your cooperation would be highly appreciated.</p>
</div>
<form method="POST" name="register_form"
action="<?php echo htmlentities($_SERVER['PHP_SELF']); ?>" class="form">
<div class="form-group">
    <label for="email">email  (optional)</label>
    <input class="form-control" type="text" name="email" value="<?php echo htmlentities($email) ?>">
</div>

<div class="form-group">
    <label for="affiliation">school/employer</label>
    <input class="form-control" type="text" name="affiliation" value="<?php echo htmlentities($affiliation) ?>">
</div>

<div class="form-group">
    <b>I plan to use OMPL for (check all that apply)</b>
    <div class="checkbox"><label><input type="checkbox" name="research" value='yes'>research</label></div>
    <div class="checkbox"><label><input type="checkbox" name="education" value='yes'>education</label></div>
    <div class="checkbox"><label><input type="checkbox" name="industry" value='yes'>industrial applications</label></div>
</div>

<div class="form-group">
    <label for="message">comments/suggestions (optional)</label>
    <textarea name="message" rows=10 cols=40 class="form-control"><?php echo htmlentities($message) ?></textarea>
</div>

<input type="submit" value="Submit" name="submit" class="btn btn-primary">

</form>
</div>

</body>
</html>
